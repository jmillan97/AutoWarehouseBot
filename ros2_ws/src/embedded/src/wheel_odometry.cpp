/*
 * wheel_odometry.cpp
 * ==================
 * Raspberry Pi ROS2 node (wb_embedded package)
 *
 * Subscribes to /left_ticks and /right_ticks published by serial_bridge,
 * computes wheel odometry, and publishes:
 *   /odom  (nav_msgs/Odometry)
 *   /tf    (odom → base_link transform)
 *
 * This node replaces fake_odometry.cpp when running on real hardware.
 * The EKF consumes /odom exactly as before — no changes needed upstream.
 *
 * Parameters (tunable via ros2 param or launch file):
 *   wheel_radius      (double)  default: 0.04   meters
 *   wheel_separation  (double)  default: 0.21   meters
 *   encoder_cpr       (double)  default: 2.0    counts per rev on motor shaft
 *   gear_ratio        (double)  default: 30.0   [TUNE] measure on real hardware
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

class WheelOdometry : public rclcpp::Node
{
public:
  WheelOdometry() : Node("wheel_odometry")
  {
    // ---- Declare parameters ----
    this->declare_parameter<double>("wheel_radius",     0.04);
    this->declare_parameter<double>("wheel_separation", 0.21);
    this->declare_parameter<double>("encoder_cpr",      2.0);
    this->declare_parameter<double>("gear_ratio",       30.0);

    // ---- Read parameters ----
    wheel_radius_     = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    double cpr        = this->get_parameter("encoder_cpr").as_double();
    double gear_ratio = this->get_parameter("gear_ratio").as_double();

    effective_cpr_   = cpr * gear_ratio;
    ticks_to_meters_ = (2.0 * M_PI * wheel_radius_) / effective_cpr_;

    RCLCPP_INFO(this->get_logger(), "wheel_odometry started");
    RCLCPP_INFO(this->get_logger(), "  wheel_radius:     %.4f m",       wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "  wheel_separation: %.4f m",       wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "  effective_cpr:    %.1f ticks/rev", effective_cpr_);
    RCLCPP_INFO(this->get_logger(), "  ticks_to_meters:  %.6f m/tick",  ticks_to_meters_);

    // ---- Subscribers ----
    left_ticks_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/left_ticks", 10,
      std::bind(&WheelOdometry::left_ticks_callback, this, std::placeholders::_1)
    );
    right_ticks_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/right_ticks", 10,
      std::bind(&WheelOdometry::right_ticks_callback, this, std::placeholders::_1)
    );

    // ---- Publishers ----
    odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---- Initialize state ----
    x_ = y_ = theta_ = 0.0;
    prev_left_ticks_ = prev_right_ticks_ = 0;
    current_left_ticks_ = current_right_ticks_ = 0;
    left_init_ = right_init_ = false;
    last_time_ = this->now();
  }

private:

  void left_ticks_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!left_init_) {
      prev_left_ticks_ = msg->data;
      left_init_ = true;
      return;
    }
    current_left_ticks_ = msg->data;
    compute_and_publish();
  }

  void right_ticks_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!right_init_) {
      prev_right_ticks_ = msg->data;
      right_init_ = true;
      return;
    }
    current_right_ticks_ = msg->data;
  }

  void compute_and_publish()
  {
    if (!left_init_ || !right_init_) return;

    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 1.0) return;

    // Delta ticks since last update
    int32_t delta_left  = current_left_ticks_  - prev_left_ticks_;
    int32_t delta_right = current_right_ticks_ - prev_right_ticks_;
    prev_left_ticks_  = current_left_ticks_;
    prev_right_ticks_ = current_right_ticks_;

    // Convert ticks → meters
    double dist_left  = delta_left  * ticks_to_meters_;
    double dist_right = delta_right * ticks_to_meters_;

    // Differential drive kinematics
    double dist_center = (dist_left + dist_right) / 2.0;
    double delta_theta = (dist_right - dist_left) / wheel_separation_;

    // Update pose
    x_     += dist_center * std::cos(theta_ + delta_theta / 2.0);
    y_     += dist_center * std::sin(theta_ + delta_theta / 2.0);
    theta_ += delta_theta;

    // Normalize to [-pi, pi]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    double vx = dist_center / dt;
    double wz = delta_theta / dt;

    // Quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // ---- Publish TF: odom → base_link ----
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = now;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // ---- Publish /odom ----
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.position.x    = x_;
    odom.pose.pose.position.y    = y_;
    odom.pose.pose.position.z    = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Covariance — [TUNE] after hardware testing
    odom.pose.covariance[0]  = 0.01;   // x
    odom.pose.covariance[7]  = 0.01;   // y
    odom.pose.covariance[35] = 0.05;   // yaw — higher without gyro

    odom.twist.twist.linear.x  = vx;
    odom.twist.twist.angular.z = wz;
    odom.twist.covariance[0]  = 0.01;
    odom.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom);
  }

  // Parameters
  double wheel_radius_;
  double wheel_separation_;
  double effective_cpr_;
  double ticks_to_meters_;

  // Subscribers / Publishers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_ticks_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_ticks_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State
  double x_, y_, theta_;
  int32_t prev_left_ticks_,    prev_right_ticks_;
  int32_t current_left_ticks_, current_right_ticks_;
  bool left_init_, right_init_;
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
