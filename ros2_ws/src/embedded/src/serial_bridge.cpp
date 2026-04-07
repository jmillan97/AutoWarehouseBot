#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

// Linux serial headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <string>
#include <thread>
#include <atomic>
#include <sstream>

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge() : Node("serial_bridge"), running_(true), serial_fd_(-1)
  {
    // ---- Parameters ----
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baud", 115200);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud         = this->get_parameter("serial_baud").as_int();

    RCLCPP_INFO(this->get_logger(), "serial_bridge starting on %s @ %d baud", port.c_str(), baud);

    // ---- Open serial port ----
    if (!open_serial(port, baud)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open serial port %s: %s",
                   port.c_str(), strerror(errno));
      throw std::runtime_error("Serial port open failed");
    }
    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

    // ---- Publishers ----
    left_ticks_pub_  = this->create_publisher<std_msgs::msg::Int32>("/left_ticks",  10);
    right_ticks_pub_ = this->create_publisher<std_msgs::msg::Int32>("/right_ticks", 10);

    // ---- Subscriber: /cmd_vel → serial ----
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SerialBridge::cmd_vel_callback, this, std::placeholders::_1)
    );

    // ---- Serial read thread ----
    // Runs in background, continuously reads lines from Arduino
    read_thread_ = std::thread(&SerialBridge::read_loop, this);

    RCLCPP_INFO(this->get_logger(), "serial_bridge ready");
  }

  ~SerialBridge()
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
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

    // Set baud rate
    speed_t speed = B115200;
    if      (baud == 9600)   speed = B9600;
    else if (baud == 57600)  speed = B57600;
    else if (baud == 115200) speed = B115200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;          // no echo, no canonical
    tty.c_oflag = 0;          // no remapping
    tty.c_cc[VMIN]  = 0;      // non-blocking read
    tty.c_cc[VTIME] = 5;      // 0.5 second read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   // no flow control
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);        // no parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;

    // Flush any garbage in buffer
    tcflush(serial_fd_, TCIOFLUSH);

    return true;
  }

  // ================================================================
  // WRITE LINE TO SERIAL
  // ================================================================
  void write_serial(const std::string & msg)
  {
    if (serial_fd_ < 0) return;
    ssize_t written = write(serial_fd_, msg.c_str(), msg.size());
    if (written < 0) {
      RCLCPP_WARN(this->get_logger(), "Serial write error: %s", strerror(errno));
    }
  }

  // ================================================================
  // cmd_vel CALLBACK → write velocity command to Arduino
  // Converts Twist (linear.x, angular.z) → wheel velocities → serial
  // ================================================================
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear  = msg->linear.x;
    double angular = msg->angular.z;

    // Differential drive kinematics
    // [TUNE] wheel_separation — must match firmware and nav2_params
    const double wheel_separation = 0.21;
    double left_vel  = linear - (angular * wheel_separation / 2.0);
    double right_vel = linear + (angular * wheel_separation / 2.0);

    // Format: "V:{left},{right}\n"
    std::ostringstream oss;
    oss << "V:" << std::fixed << std::setprecision(3)
        << left_vel << "," << right_vel << "\n";

    write_serial(oss.str());
  }

  // ================================================================
  // SERIAL READ LOOP — runs in background thread
  // Reads lines from Arduino and publishes tick counts
  // ================================================================
  void read_loop()
  {
    std::string buffer;
    char c;

    while (running_) {
      // Read one character at a time
      ssize_t n = read(serial_fd_, &c, 1);
      if (n <= 0) continue;

      if (c == '\n') {
        // Process complete line
        process_line(buffer);
        buffer.clear();
      } else if (c != '\r') {
        buffer += c;
        // Guard against runaway buffer (malformed data)
        if (buffer.size() > 64) buffer.clear();
      }
    }
  }

  // ================================================================
  // PROCESS LINE FROM ARDUINO
  // Expected format: "E:{left_ticks},{right_ticks}"
  // ================================================================
  void process_line(const std::string & line)
  {
    if (line.empty()) return;

    // Check prefix
    if (line.rfind("E:", 0) != 0) {
      // Not an encoder line — could be a debug message from Arduino
      RCLCPP_DEBUG(this->get_logger(), "Arduino: %s", line.c_str());
      return;
    }

    // Parse "E:{left},{right}"
    std::string data = line.substr(2);  // strip "E:"
    size_t comma = data.find(',');
    if (comma == std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "Malformed encoder line: %s", line.c_str());
      return;
    }

    try {
      int32_t left_ticks  = std::stoi(data.substr(0, comma));
      int32_t right_ticks = std::stoi(data.substr(comma + 1));

      // Publish
      auto left_msg  = std_msgs::msg::Int32();
      auto right_msg = std_msgs::msg::Int32();
      left_msg.data  = left_ticks;
      right_msg.data = right_ticks;

      left_ticks_pub_->publish(left_msg);
      right_ticks_pub_->publish(right_msg);

    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse encoder line '%s': %s",
                  line.c_str(), e.what());
    }
  }

  // ---- Members ----
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
