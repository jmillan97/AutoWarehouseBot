/*
 * serial_bridge.cpp
 * =================
 * Raspberry Pi ROS2 node — bridges /cmd_vel to Arduino serial
 *
 * Protocol:
 *   Pi → Arduino:   "V:{linear_mps},{angular_rads}\n"
 *   Arduino → Pi:   "E:{left_ticks},{right_ticks}\n"
 *
 * Subscribes:  /cmd_vel  (geometry_msgs/Twist or TwistStamped)
 * Publishes:   /left_ticks  (std_msgs/Int32)
 *              /right_ticks (std_msgs/Int32)
 *
 * Parameters:
 *   serial_port  (string)  default: /dev/ttyUSB0
 *   serial_baud  (int)     default: 9600
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge() : Node("serial_bridge"), running_(true), serial_fd_(-1)
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baud", 115200);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud         = this->get_parameter("serial_baud").as_int();

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

    // Subscribe to /cmd_vel — Nav2 publishes Twist here
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SerialBridge::cmd_vel_callback, this, std::placeholders::_1)
    );

    // Background thread reads encoder lines from Arduino
    read_thread_ = std::thread(&SerialBridge::read_loop, this);

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

  // ================================================================
  // /cmd_vel CALLBACK
  // Converts Twist → "V:{linear},{angular}\n" → Arduino
  // ================================================================
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "V:"
        << std::fixed << std::setprecision(4) << msg->linear.x
        << ","
        << std::fixed << std::setprecision(4) << msg->angular.z
        << "\n";

    write_serial(oss.str());
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

  // Members
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_ticks_pub_;

  int serial_fd_;
  std::atomic<bool> running_;
  std::thread read_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridge>());
  rclcpp::shutdown();
  return 0;
}
