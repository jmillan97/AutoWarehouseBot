/*
 * serial_bridge.cpp
 * =================
 * Raspberry Pi ROS2 node — bridges ROS movement topics to legacy Arduino
 * single-character commands.
 *
 * Legacy protocol expected by known-good firmware:
 *   Pi → Arduino: "speed:{0-255}\n", "w\n", "s\n", "q\n", "e\n", "x\n", "r\n"
 *   Arduino → Pi: "E:{left_ticks},{right_ticks}\n"
 *
 * ROS command API:
 *   /move_distance_mm (std_msgs/Int32)  signed mm, +forward -backward
 *   /rotate_angle_deg (std_msgs/Int32)  signed deg, +ccw -cw
 *
 * Feedback:
 *   /left_ticks, /right_ticks
 *
 * This preserves old firmware motor behavior while enabling deterministic
 * movement primitives from ROS.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

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
#include <cmath>
#include <chrono>
#include <cstdlib>

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge() : Node("serial_bridge"), serial_fd_(-1), running_(true)
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baud", 115200);
    this->declare_parameter<int>("command_speed", 90);
    this->declare_parameter<double>("wheel_radius", 0.04);
    this->declare_parameter<double>("wheel_separation", 0.21);
    this->declare_parameter<double>("encoder_cpr", 2.0);
    this->declare_parameter<double>("gear_ratio", 108.0);
    this->declare_parameter<double>("command_timeout_s", 15.0);
    this->declare_parameter<double>("command_rate_hz", 10.0);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud         = this->get_parameter("serial_baud").as_int();
    command_speed_   = this->get_parameter("command_speed").as_int();
    wheel_radius_    = this->get_parameter("wheel_radius").as_double();
    wheel_sep_       = this->get_parameter("wheel_separation").as_double();
    encoder_cpr_     = this->get_parameter("encoder_cpr").as_double();
    gear_ratio_      = this->get_parameter("gear_ratio").as_double();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();
    command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();

    if (command_speed_ < 0) command_speed_ = 0;
    if (command_speed_ > 255) command_speed_ = 255;

    const double effective_cpr = encoder_cpr_ * gear_ratio_;
    ticks_to_meters_ = (2.0 * M_PI * wheel_radius_) / effective_cpr;

    RCLCPP_INFO(this->get_logger(), "Opening %s @ %d baud", port.c_str(), baud);

    if (!open_serial(port, baud)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s: %s",
                   port.c_str(), strerror(errno));
      throw std::runtime_error("Serial port open failed");
    }

    RCLCPP_INFO(this->get_logger(), "Serial port open — waiting for ACK:SYSTEM_READY");

    // Publishers
    left_ticks_pub_  = this->create_publisher<std_msgs::msg::Int32>("/left_ticks",  10);
    right_ticks_pub_ = this->create_publisher<std_msgs::msg::Int32>("/right_ticks", 10);

    // Exact-distance command in millimeters (signed): +mm forward, -mm backward
    move_mm_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/move_distance_mm", 10,
      std::bind(&SerialBridge::move_mm_callback, this, std::placeholders::_1)
    );

    // Exact-rotation command in degrees (signed): +deg CCW, -deg CW
    rotate_deg_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/rotate_angle_deg", 10,
      std::bind(&SerialBridge::rotate_deg_callback, this, std::placeholders::_1)
    );

    // Background thread reads encoder lines from Arduino
    read_thread_ = std::thread(&SerialBridge::read_loop, this);

    // Control loop keeps legacy command alive until encoder target is reached.
    const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SerialBridge::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "serial_bridge ready");
  }

  ~SerialBridge()
  {
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:

  // ================================================================
  // OPEN SERIAL PORT
  // ================================================================
  bool open_serial(const std::string & port, int baud)
  {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    speed_t speed = B9600;
    if      (baud == 9600)   speed = B9600;
    else if (baud == 57600)  speed = B57600;
    else if (baud == 115200) speed = B115200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
  }

  // ================================================================
  // WRITE TO SERIAL
  // ================================================================
  void write_serial(const std::string & msg)
  {
    if (serial_fd_ < 0) return;
    ssize_t n = write(serial_fd_, msg.c_str(), msg.size());
    if (n < 0) {
      RCLCPP_WARN(this->get_logger(), "Serial write error: %s", strerror(errno));
    }
  }

  void send_legacy_command(const char * cmd)
  {
    std::string msg(cmd);
    msg.push_back('\n');
    write_serial(msg);
  }

  void send_speed_once()
  {
    std::ostringstream oss;
    oss << "speed:" << command_speed_ << "\n";
    write_serial(oss.str());
  }

  void move_mm_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int value = msg->data;
    if (value == 0) {
      cancel_motion("Received zero distance command");
      return;
    }
    start_motion(MotionType::Linear, value);
  }

  void rotate_deg_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int value = msg->data;
    if (value == 0) {
      cancel_motion("Received zero rotation command");
      return;
    }
    start_motion(MotionType::Rotate, value);
  }

  // ================================================================
  // SERIAL READ LOOP — background thread
  // Reads "E:{left},{right}\n" from Arduino
  // ================================================================
  void read_loop()
  {
    std::string buffer;
    char c;

    while (running_) {
      ssize_t n = read(serial_fd_, &c, 1);
      if (n <= 0) continue;

      if (c == '\n') {
        if (!buffer.empty()) {
          process_line(buffer);
          buffer.clear();
        }
      } else if (c != '\r') {
        buffer += c;
        if (buffer.size() > 64) buffer.clear();
      }
    }
  }

  // ================================================================
  // PROCESS LINE FROM ARDUINO
  // "E:{left_ticks},{right_ticks}"  → publish to ROS2
  // "ACK:..."                        → log info
  // ================================================================
  void process_line(const std::string & line)
  {
    if (line.empty()) return;

    // ACK messages from Arduino — just log them
    if (line.rfind("ACK:", 0) == 0) {
      RCLCPP_INFO(this->get_logger(), "Arduino: %s", line.c_str());
      return;
    }

    // Encoder lines
    if (line.rfind("E:", 0) != 0) {
      RCLCPP_DEBUG(this->get_logger(), "Arduino: %s", line.c_str());
      return;
    }

    std::string data = line.substr(2);
    size_t comma = data.find(',');
    if (comma == std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "Malformed encoder line: %s", line.c_str());
      return;
    }

    try {
      int32_t left_ticks  = std::stoi(data.substr(0, comma));
      int32_t right_ticks = std::stoi(data.substr(comma + 1));

      {
        std::lock_guard<std::mutex> lk(motion_mtx_);
        current_left_ticks_ = left_ticks;
        current_right_ticks_ = right_ticks;
      }

      auto left_msg  = std_msgs::msg::Int32();
      auto right_msg = std_msgs::msg::Int32();
      left_msg.data  = left_ticks;
      right_msg.data = right_ticks;

      left_ticks_pub_->publish(left_msg);
      right_ticks_pub_->publish(right_msg);

    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Parse error on '%s': %s",
                  line.c_str(), e.what());
    }
  }

  enum class MotionType { None, Linear, Rotate };

  void start_motion(MotionType type, int signed_value)
  {
    std::lock_guard<std::mutex> lk(motion_mtx_);

    const int direction = (signed_value > 0) ? 1 : -1;
    const int abs_value = std::abs(signed_value);

    long target_ticks = 0;
    if (type == MotionType::Linear) {
      const double meters = static_cast<double>(abs_value) / 1000.0;
      target_ticks = static_cast<long>(std::lround(meters / ticks_to_meters_));
    } else {
      const double theta_rad = static_cast<double>(abs_value) * M_PI / 180.0;
      const double wheel_arc = (wheel_sep_ / 2.0) * theta_rad;
      target_ticks = static_cast<long>(std::lround(wheel_arc / ticks_to_meters_));
    }
    if (target_ticks < 1) target_ticks = 1;

    motion_active_ = true;
    motion_type_ = type;
    motion_direction_ = direction;
    motion_target_ticks_ = target_ticks;
    motion_start_left_ticks_ = current_left_ticks_;
    motion_start_right_ticks_ = current_right_ticks_;
    motion_start_time_ = this->now();
    speed_sent_for_motion_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Motion start: type=%s value=%d target_ticks=%ld dir=%d",
      (type == MotionType::Linear ? "linear" : "rotate"),
      signed_value,
      target_ticks,
      direction
    );
  }

  void cancel_motion(const char * reason)
  {
    std::lock_guard<std::mutex> lk(motion_mtx_);
    motion_active_ = false;
    motion_type_ = MotionType::None;
    motion_direction_ = 0;
    motion_target_ticks_ = 0;
    send_legacy_command("x");
    RCLCPP_INFO(this->get_logger(), "Motion cancelled: %s", reason);
  }

  void control_loop()
  {
    std::lock_guard<std::mutex> lk(motion_mtx_);
    if (!motion_active_) return;

    const auto now = this->now();
    const double elapsed = (now - motion_start_time_).seconds();
    if (elapsed > command_timeout_s_) {
      send_legacy_command("x");
      RCLCPP_WARN(this->get_logger(), "Motion timeout after %.2fs", elapsed);
      motion_active_ = false;
      motion_type_ = MotionType::None;
      return;
    }

    const long dleft = std::labs(current_left_ticks_ - motion_start_left_ticks_);
    const long dright = std::labs(current_right_ticks_ - motion_start_right_ticks_);
    const long traveled = (dleft + dright) / 2;

    if (traveled >= motion_target_ticks_) {
      send_legacy_command("x");
      RCLCPP_INFO(this->get_logger(), "Motion complete: traveled=%ld target=%ld", traveled, motion_target_ticks_);
      motion_active_ = false;
      motion_type_ = MotionType::None;
      return;
    }

    if (!speed_sent_for_motion_) {
      send_speed_once();
      speed_sent_for_motion_ = true;
    }

    if (motion_type_ == MotionType::Linear) {
      if (motion_direction_ > 0) send_legacy_command("w");
      else send_legacy_command("s");
    } else if (motion_type_ == MotionType::Rotate) {
      if (motion_direction_ > 0) send_legacy_command("q");
      else send_legacy_command("e");
    }
  }

  // Members
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr move_mm_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotate_deg_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_ticks_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Motion model / command params
  int command_speed_{90};
  double wheel_radius_{0.04};
  double wheel_sep_{0.21};
  double encoder_cpr_{2.0};
  double gear_ratio_{108.0};
  double ticks_to_meters_{0.0};
  double command_timeout_s_{15.0};
  double command_rate_hz_{10.0};

  int serial_fd_;
  std::atomic<bool> running_;
  std::thread read_thread_;

  // Shared encoder state + active motion control
  std::mutex motion_mtx_;
  int32_t current_left_ticks_{0};
  int32_t current_right_ticks_{0};
  bool motion_active_{false};
  MotionType motion_type_{MotionType::None};
  int motion_direction_{0};
  long motion_target_ticks_{0};
  int32_t motion_start_left_ticks_{0};
  int32_t motion_start_right_ticks_{0};
  rclcpp::Time motion_start_time_;
  bool speed_sent_for_motion_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridge>());
  rclcpp::shutdown();
  return 0;
}
