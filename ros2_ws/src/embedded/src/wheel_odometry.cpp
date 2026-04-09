/**
 * fake_odometry.cpp
 * =================
 * Publishes synthetic odometry on /odom at 30Hz.
 * Simulates a stationary robot — near-zero velocity with small noise.
 * Gives robot_localization EKF something to fuse before hardware arrives.
 *
 * Topic:  /odom  [nav_msgs/Odometry]
 * Frame:  odom -> base_link
 * Rate:   30Hz
 */

#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class FakeOdometry : public rclcpp::Node
{
public:
  FakeOdometry()
  : Node("fake_odometry"),
    rng_(std::random_device{}())
  {
    publish_rate_ = declare_parameter("publish_rate", 30.0);
    noise_stddev_ = declare_parameter("noise_stddev", 0.0001);

    noise_ = std::normal_distribution<double>(0.0, noise_stddev_);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(period_ms,
      std::bind(&FakeOdometry::publish_odom, this));

    RCLCPP_INFO(get_logger(), "FakeOdometry running at %.0f Hz", publish_rate_);
  }

private:
  void publish_odom()
  {
    auto now = this->now();

    // ---- Odometry message ----
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp    = now;
    msg.header.frame_id = "odom";
    msg.child_frame_id  = "base_link";

    // Position — stationary robot at origin
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    // Orientation — identity quaternion
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;

    // Pose covariance — diagonal, moderate uncertainty
    msg.pose.covariance[0]  = 0.01;   // x
    msg.pose.covariance[7]  = 0.01;   // y
    msg.pose.covariance[35] = 0.01;   // yaw

    // Velocity — near zero with noise
    msg.twist.twist.linear.x  = noise_(rng_);
    msg.twist.twist.angular.z = noise_(rng_);

    msg.twist.covariance[0]  = 0.01;
    msg.twist.covariance[35] = 0.01;

    publisher_->publish(msg);

    // ---- TF broadcast: odom -> base_link ----
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = now;
    tf.header.frame_id         = "odom";
    tf.child_frame_id          = "base_link";
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w    = 1.0;
    tf.transform.rotation.x    = 0.0;
    tf.transform.rotation.y    = 0.0;
    tf.transform.rotation.z    = 0.0;

    tf_broadcaster_->sendTransform(tf);
  }

  double publish_rate_;
  double noise_stddev_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdometry>());
  rclcpp::shutdown();
  return 0;
}