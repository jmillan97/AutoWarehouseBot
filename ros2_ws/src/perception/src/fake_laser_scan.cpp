/**
 * fake_laser_scan.cpp
 * ===================
 * Publishes a synthetic 360-degree LaserScan on /scan.
 *
 * Purpose: Allows Nav2, SLAM, and costmap nodes to launch and be tested
 * BEFORE physical LiDAR hardware is available. Simulates a clear room
 * (all ranges at max) with configurable obstacle injection for testing.
 *
 * Topic:  /scan  [sensor_msgs/LaserScan]
 * Frame:  base_laser
 * Rate:   10 Hz (matches real RPLIDAR)
 *
 * ROS2 Parameters:
 *   ~range_max      (double, default 12.0)  — sensor max range
 *   ~range_min      (double, default 0.15)  — sensor min range
 *   ~num_readings   (int,    default 360)   — samples per revolution
 *   ~publish_rate   (double, default 10.0)  — Hz
 *   ~inject_obstacle (bool,  default false) — add a fake wall for testing
 *   ~obstacle_angle (double, default 0.0)   — angle of fake obstacle (rad)
 *   ~obstacle_range (double, default 1.5)   — range of fake obstacle (m)
 *   ~obstacle_width (int,    default 20)    — angular width in samples
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class FakeLaserScan : public rclcpp::Node
{
public:
  FakeLaserScan()
  : Node("fake_laser_scan")
  {
    // ---- Declare parameters ----
    range_max_       = declare_parameter("range_max",       12.0);
    range_min_       = declare_parameter("range_min",       0.15);
    num_readings_    = declare_parameter("num_readings",    360);
    publish_rate_    = declare_parameter("publish_rate",    10.0);
    inject_obstacle_ = declare_parameter("inject_obstacle", false);
    obstacle_angle_  = declare_parameter("obstacle_angle",  0.0);
    obstacle_range_  = declare_parameter("obstacle_range",  1.5);
    obstacle_width_  = declare_parameter("obstacle_width",  20);

    // ---- Publisher ----
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // ---- Timer ----
    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(period_ms,
      std::bind(&FakeLaserScan::publish_scan, this));

    RCLCPP_INFO(get_logger(),
      "FakeLaserScan running — %d rays, %.1f Hz, obstacle_inject=%s",
      num_readings_, publish_rate_,
      inject_obstacle_ ? "ON" : "OFF");
  }

private:
  void publish_scan()
  {
    auto msg = sensor_msgs::msg::LaserScan();

    // Header — frame must match interfaces.md: base_laser
    msg.header.stamp    = now();
    msg.header.frame_id = "base_laser";

    // Scan geometry: full 360 degrees
    msg.angle_min       = -M_PI;
    msg.angle_max       =  M_PI;
    msg.angle_increment = (2.0 * M_PI) / static_cast<double>(num_readings_);
    msg.time_increment  = 0.0;
    msg.scan_time       = 1.0 / publish_rate_;
    msg.range_min       = range_min_;
    msg.range_max       = range_max_;

    // Default: clear room — all ranges at max
    msg.ranges.assign(num_readings_, static_cast<float>(range_max_));
    msg.intensities.assign(num_readings_, 100.0f);

    // Optional: inject a fake obstacle for testing planner/costmap
    if (inject_obstacle_) {
      inject_fake_obstacle(msg);
    }

    publisher_->publish(msg);
  }

  void inject_fake_obstacle(sensor_msgs::msg::LaserScan & msg)
  {
    // Convert obstacle_angle_ to scan index
    int center_idx = static_cast<int>(
      (obstacle_angle_ - msg.angle_min) / msg.angle_increment);

    int half_w = obstacle_width_ / 2;

    for (int i = center_idx - half_w; i <= center_idx + half_w; ++i) {
      // Wrap around 360
      int idx = ((i % num_readings_) + num_readings_) % num_readings_;
      msg.ranges[idx] = static_cast<float>(obstacle_range_);
    }
  }

  // Parameters
  double range_max_;
  double range_min_;
  int    num_readings_;
  double publish_rate_;
  bool   inject_obstacle_;
  double obstacle_angle_;
  double obstacle_range_;
  int    obstacle_width_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLaserScan>());
  rclcpp::shutdown();
  return 0;
}
