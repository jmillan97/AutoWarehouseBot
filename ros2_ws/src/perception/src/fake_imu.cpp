/**
 * fake_imu.cpp
 * ============
 * Publishes synthetic IMU data on /imu/data.
 *
 * Purpose: Allows robot_localization EKF and Nav2 to start before
 * physical IMU hardware is connected. Publishes near-zero values with
 * small Gaussian noise to simulate a stationary robot.
 *
 * Topic:  /imu/data  [sensor_msgs/Imu]
 * Frame:  base_link  (IMU is assumed rigidly mounted to chassis)
 * Rate:   50 Hz (typical IMU rate)
 *
 * ROS2 Parameters:
 *   ~publish_rate       (double, default 50.0)   — Hz
 *   ~noise_stddev       (double, default 0.001)  — rad/s noise on gyro
 *   ~accel_noise_stddev (double, default 0.01)   — m/s^2 noise on accel
 *   ~gravity            (double, default 9.81)   — m/s^2 gravity constant
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class FakeImu : public rclcpp::Node
{
public:
  FakeImu()
  : Node("fake_imu"),
    rng_(std::random_device{}())
  {
    // ---- Parameters ----
    publish_rate_       = declare_parameter("publish_rate",       50.0);
    noise_stddev_       = declare_parameter("noise_stddev",       0.001);
    accel_noise_stddev_ = declare_parameter("accel_noise_stddev", 0.01);
    gravity_            = declare_parameter("gravity",            9.81);

    gyro_noise_  = std::normal_distribution<double>(0.0, noise_stddev_);
    accel_noise_ = std::normal_distribution<double>(0.0, accel_noise_stddev_);

    // ---- Publisher ----
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // ---- Timer ----
    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(period_ms,
      std::bind(&FakeImu::publish_imu, this));

    RCLCPP_INFO(get_logger(),
      "FakeImu running — %.0f Hz, gyro_noise=%.4f, accel_noise=%.4f",
      publish_rate_, noise_stddev_, accel_noise_stddev_);
  }

private:
  void publish_imu()
  {
    auto msg = sensor_msgs::msg::Imu();

    msg.header.stamp    = now();
    msg.header.frame_id = "base_link";

    // Orientation: identity quaternion (robot starts level)
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;

    // Orientation covariance — moderate uncertainty for fake data
    // Row-major 3x3 matrix [roll, pitch, yaw]
    msg.orientation_covariance = {
      0.01, 0.0,  0.0,
      0.0,  0.01, 0.0,
      0.0,  0.0,  0.01
    };

    // Angular velocity: near-zero with noise (stationary robot)
    msg.angular_velocity.x = gyro_noise_(rng_);
    msg.angular_velocity.y = gyro_noise_(rng_);
    msg.angular_velocity.z = gyro_noise_(rng_);

    msg.angular_velocity_covariance = {
      noise_stddev_ * noise_stddev_, 0.0, 0.0,
      0.0, noise_stddev_ * noise_stddev_, 0.0,
      0.0, 0.0, noise_stddev_ * noise_stddev_
    };

    // Linear acceleration: gravity on Z axis with noise
    // Robot is assumed flat — gravity is in the -Z direction of world,
    // which is +Z in the robot's IMU frame when robot is upright.
    msg.linear_acceleration.x = accel_noise_(rng_);
    msg.linear_acceleration.y = accel_noise_(rng_);
    msg.linear_acceleration.z = gravity_ + accel_noise_(rng_);

    msg.linear_acceleration_covariance = {
      accel_noise_stddev_ * accel_noise_stddev_, 0.0, 0.0,
      0.0, accel_noise_stddev_ * accel_noise_stddev_, 0.0,
      0.0, 0.0, accel_noise_stddev_ * accel_noise_stddev_
    };

    publisher_->publish(msg);
  }

  // Parameters
  double publish_rate_;
  double noise_stddev_;
  double accel_noise_stddev_;
  double gravity_;

  // RNG
  std::mt19937 rng_;
  std::normal_distribution<double> gyro_noise_;
  std::normal_distribution<double> accel_noise_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImu>());
  rclcpp::shutdown();
  return 0;
}
