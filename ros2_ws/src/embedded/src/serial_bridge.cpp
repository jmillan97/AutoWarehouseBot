#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <string>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// ================================================================
// Helpers
// ================================================================

static speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B115200;
  }
}

// ================================================================
// Node
// ================================================================

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge()
  : Node("serial_bridge"),
    running_(true),
    serial_fd_(-1),
    last_cmd_time_(this->now()),
    // Odometry state
    x_(0.0), y_(0.0), theta_(0.0),
    prev_left_ticks_(0), prev_right_ticks_(0),
    first_encoder_(true)
  {
    // ── Parameters ──────────────────────────────────────────────
    this->declare_parameter<std::string>("serial_port",    "/dev/arduino");
    this->declare_parameter<int>        ("serial_baud",    115200);
    this->declare_parameter<int>        ("fixed_pwm",      80);
    this->declare_parameter<double>     ("angular_thresh", 0.05);
    this->declare_parameter<double>     ("linear_thresh",  0.05);
    this->declare_parameter<int>        ("cmd_timeout_ms", 300);

    // Odometry parameters — must match physical robot
    this->declare_parameter<double>("wheel_radius",      0.04);
    this->declare_parameter<double>("wheel_separation",   0.21);
    this->declare_parameter<double>("encoder_cpr",        2.0);
    this->declare_parameter<double>("gear_ratio",         30.0);

    // Steering correction (from Python bridge calibration)
    this->declare_parameter<double>("linear_balance_kp",  0.4);
    this->declare_parameter<double>("linear_steer_bias", -6.0);

    // Whether to publish odom->base_link TF from this node
    this->declare_parameter<bool>("publish_odom_tf", true);

    std::string port = this->get_parameter("serial_port").as_string();
    int         baud = this->get_parameter("serial_baud").as_int();
    fixed_pwm_       = this->get_parameter("fixed_pwm").as_int();
    angular_thresh_  = this->get_parameter("angular_thresh").as_double();
    linear_thresh_   = this->get_parameter("linear_thresh").as_double();
    cmd_timeout_ms_  = this->get_parameter("cmd_timeout_ms").as_int();

    wheel_radius_    = this->get_parameter("wheel_radius").as_double();
    wheel_sep_       = this->get_parameter("wheel_separation").as_double();
    encoder_cpr_     = this->get_parameter("encoder_cpr").as_double();
    gear_ratio_      = this->get_parameter("gear_ratio").as_double();
    balance_kp_      = this->get_parameter("linear_balance_kp").as_double();
    steer_bias_      = this->get_parameter("linear_steer_bias").as_double();
    publish_odom_tf_ = this->get_parameter("publish_odom_tf").as_bool();

    // Derived
    double effective_cpr = encoder_cpr_ * gear_ratio_;
    meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / effective_cpr;

    RCLCPP_INFO(this->get_logger(),
      "serial_bridge  port=%s  baud=%d  fixed_pwm=%d  m/tick=%.6f",
      port.c_str(), baud, fixed_pwm_, meters_per_tick_);

    // ── Open serial ─────────────────────────────────────────────
    if (!open_serial(port, baud)) {
      RCLCPP_FATAL(this->get_logger(), "Cannot open serial port %s: %s",
                   port.c_str(), strerror(errno));
      throw std::runtime_error("Serial port open failed");
    }
    RCLCPP_INFO(this->get_logger(), "Serial port open");

    // ── Publishers ──────────────────────────────────────────────
    left_ticks_pub_  = this->create_publisher<std_msgs::msg::Int32>("/left_ticks",  10);
    right_ticks_pub_ = this->create_publisher<std_msgs::msg::Int32>("/right_ticks", 10);
    odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

    // ── TF broadcaster ──────────────────────────────────────────
    if (publish_odom_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // ── /cmd_vel subscriber ─────────────────────────────────────
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        handle_cmd_vel(msg->twist.linear.x, msg->twist.angular.z);
      });

    // ── Watchdog — zeros motors if /cmd_vel goes silent ─────────
    watchdog_timer_ = this->create_wall_timer(
      50ms,
      [this]() { check_cmd_timeout(); });

    // ── Background thread — reads encoder lines from Arduino ────
    read_thread_ = std::thread([this]() { read_loop(); });

    RCLCPP_INFO(this->get_logger(), "serial_bridge ready (with odometry)");
  }

  ~SerialBridge()
  {
    running_ = false;
    send_drive(0, 0);
    if (read_thread_.joinable()) read_thread_.join();
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  // ── /cmd_vel callback ─────────────────────────────────────────
  void handle_cmd_vel(double linear, double angular)
  {
    last_cmd_time_ = this->now();

    int left_pwm  = 0;
    int right_pwm = 0;

    if (std::abs(angular) >= angular_thresh_) {
      // ── Rotate in place — angular always wins ─────────────────
      int sign  = (angular > 0.0) ? 1 : -1;
      left_pwm  = -sign * fixed_pwm_;
      right_pwm =  sign * fixed_pwm_;

    } else if (std::abs(linear) >= linear_thresh_) {
      // ── Drive straight with steering correction ───────────────
      int sign = (linear > 0.0) ? 1 : -1;

      // Apply steering bias and balance correction from encoder feedback
      double tick_error = 0.0;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        // Use cumulative tick difference as steering error
        tick_error = static_cast<double>(prev_left_ticks_ - prev_right_ticks_);
      }
      double correction = balance_kp_ * tick_error;

      left_pwm  = static_cast<int>((fixed_pwm_ - correction + steer_bias_) * sign);
      right_pwm = static_cast<int>((fixed_pwm_ + correction - steer_bias_) * sign);

      // Clamp to valid PWM range
      left_pwm  = std::max(-255, std::min(255, left_pwm));
      right_pwm = std::max(-255, std::min(255, right_pwm));
    }
    // else both below threshold → stop (0, 0)

    send_drive(left_pwm, right_pwm);
  }

  // ── Watchdog ──────────────────────────────────────────────────
  void check_cmd_timeout()
  {
    auto elapsed_ms = (this->now() - last_cmd_time_).nanoseconds() / 1'000'000;
    if (elapsed_ms > cmd_timeout_ms_) {
      send_drive(0, 0);
    }
  }

  // ── Write "drive_lr:{left},{right}\n" to Arduino ──────────────
  void send_drive(int left_pwm, int right_pwm)
  {
    if (serial_fd_ < 0) return;

    std::ostringstream oss;
    oss << "drive_lr:" << left_pwm << "," << right_pwm << "\n";
    std::string cmd = oss.str();

    ssize_t written = write(serial_fd_, cmd.c_str(), cmd.size());
    if (written < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Serial write failed: %s", strerror(errno));
    }
  }

  // ── Background encoder read loop ──────────────────────────────
  void read_loop()
  {
    std::string buf;
    buf.reserve(64);
    char c;

    while (running_) {
      ssize_t n = read(serial_fd_, &c, 1);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(1ms);
          continue;
        }
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
        break;
      }
      if (n == 0) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      if (c == '\n') {
        parse_serial_line(buf);
        buf.clear();
      } else if (c != '\r') {
        buf += c;
        if (buf.size() > 128) buf.clear();
      }
    }
  }

  // ── Parse "E:{front_right},{rear_left}" from Arduino ──────────
  //
  // Arduino firmware prints: E:<ticksFR>,<ticksRL>
  //   ticksFR = front-right encoder (M1 side)
  //   ticksRL = rear-left encoder  (M4 side)
  //
  // For differential drive odometry we treat:
  //   right_ticks = ticksFR  (first value)
  //   left_ticks  = ticksRL  (second value)
  //
  // This matches the Python bridge convention.
  // ────────────────────────────────────────────────────────────────
  void parse_serial_line(const std::string & line)
  {
    if (line.size() < 4 || line[0] != 'E' || line[1] != ':') {
      if (!line.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Arduino: %s", line.c_str());
      }
      return;
    }

    try {
      size_t comma = line.find(',', 2);
      if (comma == std::string::npos) return;

      // Arduino sends E:<FR>,<RL>  →  FR=right side, RL=left side
      int32_t right_ticks = std::stoi(line.substr(2, comma - 2));
      int32_t left_ticks  = std::stoi(line.substr(comma + 1));

      // Publish raw ticks
      auto lmsg = std_msgs::msg::Int32();
      auto rmsg = std_msgs::msg::Int32();
      lmsg.data = left_ticks;
      rmsg.data = right_ticks;
      left_ticks_pub_->publish(lmsg);
      right_ticks_pub_->publish(rmsg);

      // Update odometry
      update_odometry(left_ticks, right_ticks);
    }
    catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Bad encoder line [%s]: %s",
                  line.c_str(), e.what());
    }
  }

  // ── Wheel odometry calculation ────────────────────────────────
  void update_odometry(int32_t left_ticks, int32_t right_ticks)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    if (first_encoder_) {
      prev_left_ticks_  = left_ticks;
      prev_right_ticks_ = right_ticks;
      last_odom_time_   = this->now();
      first_encoder_    = false;
      return;
    }

    // Tick deltas
    int32_t dl = left_ticks  - prev_left_ticks_;
    int32_t dr = right_ticks - prev_right_ticks_;
    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;

    // Distance traveled by each wheel
    double dist_left  = dl * meters_per_tick_;
    double dist_right = dr * meters_per_tick_;

    // Differential drive kinematics
    double dist_center = (dist_left + dist_right) / 2.0;
    double dtheta      = (dist_right - dist_left) / wheel_sep_;

    // Update pose
    theta_ += dtheta;
    // Normalize theta to [-pi, pi]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    x_ += dist_center * std::cos(theta_);
    y_ += dist_center * std::sin(theta_);

    // Compute velocities
    auto now = this->now();
    double dt = (now - last_odom_time_).seconds();
    last_odom_time_ = now;

    double vx = 0.0;
    double vtheta = 0.0;
    if (dt > 0.0 && dt < 1.0) {
      vx     = dist_center / dt;
      vtheta = dtheta / dt;
    }

    // Build quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // ── Publish /odom ─────────────────────────────────────────
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Covariance — moderate uncertainty
    // [0]=x, [7]=y, [35]=yaw
    odom_msg.pose.covariance[0]  = 0.01;
    odom_msg.pose.covariance[7]  = 0.01;
    odom_msg.pose.covariance[35] = 0.03;

    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.angular.z = vtheta;

    odom_msg.twist.covariance[0]  = 0.01;
    odom_msg.twist.covariance[7]  = 0.01;
    odom_msg.twist.covariance[35] = 0.03;

    odom_pub_->publish(odom_msg);

    // ── Publish odom → base_link TF ─────────────────────────────
    if (publish_odom_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp    = now;
      t.header.frame_id = "odom";
      t.child_frame_id  = "base_link";
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);
    }
  }

  // ── Open and configure serial port ────────────────────────────
  bool open_serial(const std::string & port, int baud)
  {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) return false;

    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    speed_t spd = baud_constant(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
  }

  // ── Members ───────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_ticks_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::atomic<bool>  running_;
  std::thread        read_thread_;
  int                serial_fd_;
  rclcpp::Time       last_cmd_time_;

  int    fixed_pwm_;
  double angular_thresh_;
  double linear_thresh_;
  int    cmd_timeout_ms_;

  // Odometry parameters
  double wheel_radius_;
  double wheel_sep_;
  double encoder_cpr_;
  double gear_ratio_;
  double meters_per_tick_;
  double balance_kp_;
  double steer_bias_;
  bool   publish_odom_tf_;

  // Odometry state (protected by odom_mutex_)
  std::mutex odom_mutex_;
  double x_, y_, theta_;
  int32_t prev_left_ticks_, prev_right_ticks_;
  bool first_encoder_;
  rclcpp::Time last_odom_time_;
};

// ================================================================
// main
// ================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SerialBridge>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_bridge"), "Fatal: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}