/*
 * serial_bridge.cpp
 * =================
 * Raspberry Pi ROS2 node  (wb_embedded package)
 * AutoWarehouseBot — ros2 branch
 *
 * Bridges /cmd_vel  →  Arduino serial  →  /left_ticks, /right_ticks
 *
 * ── Motion Model ─────────────────────────────────────────────────
 *  Robot moves at a fixed PWM of 80. No variable speed.
 *  Angular always has priority over linear:
 *    - angular.z != 0  → rotate in place  (left=-80, right=+80 or inverse)
 *    - linear.x  != 0  → drive straight   (left=+80, right=+80 or inverse)
 *    - both nonzero    → angular wins, linear ignored until angular settles
 *
 * ── Serial Protocol ──────────────────────────────────────────────
 *  Pi → Arduino:   "drive_lr:{left_pwm},{right_pwm}\n"
 *                   values are -80, 0, or +80 only
 *
 *  Arduino → Pi:   "E:{left_ticks},{right_ticks}\n"
 *                   cumulative int32 encoder counts
 *
 * ── ROS2 Interface ───────────────────────────────────────────────
 *  Subscribes:  /cmd_vel        geometry_msgs/Twist
 *  Publishes:   /left_ticks     std_msgs/Int32
 *               /right_ticks    std_msgs/Int32
 *
 * ── Parameters ───────────────────────────────────────────────────
 *  serial_port      string   /dev/arduino
 *  serial_baud      int      115200
 *  fixed_pwm        int      80       PWM magnitude (always this value, never scaled)
 *  angular_thresh   double   0.05     min |angular.z| rad/s to trigger rotate
 *  linear_thresh    double   0.05     min |linear.x|  m/s  to trigger drive
 *  cmd_timeout_ms   int      300      zero motors if /cmd_vel silent this long [ms]
 *
 * ── udev rule (run once on Pi) ───────────────────────────────────
 *  1. Find your Arduino's vendor/product ID:
 *       udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
 *       (Uno R3 clone is usually 1a86:7523, genuine Uno is 2341:0043)
 *
 *  2. Create /etc/udev/rules.d/99-arduino.rules:
 *       SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
 *       SYMLINK+="arduino", MODE="0666"
 *     (replace idVendor/idProduct with values from step 1)
 *
 *  3. Reload:
 *       sudo udevadm control --reload-rules && sudo udevadm trigger
 *
 *  4. Verify:
 *       ls -la /dev/arduino   # should show symlink to /dev/ttyUSB0 or ttyACM0
 *
 * ── Build ─────────────────────────────────────────────────────────
 *  Part of wb_embedded package.
 *  ros2 run wb_embedded serial_bridge
 *  ros2 run wb_embedded serial_bridge --ros-args -p fixed_pwm:=60
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
#include <thread>
#include <atomic>
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
    last_cmd_time_(this->now())
  {
    // ── Parameters ──────────────────────────────────────────────
    this->declare_parameter<std::string>("serial_port",    "/dev/arduino");
    this->declare_parameter<int>        ("serial_baud",    115200);
    this->declare_parameter<int>        ("fixed_pwm",      80);
    this->declare_parameter<double>     ("angular_thresh", 0.05);
    this->declare_parameter<double>     ("linear_thresh",  0.05);
    this->declare_parameter<int>        ("cmd_timeout_ms", 300);

    std::string port = this->get_parameter("serial_port").as_string();
    int         baud = this->get_parameter("serial_baud").as_int();
    fixed_pwm_       = this->get_parameter("fixed_pwm").as_int();
    angular_thresh_  = this->get_parameter("angular_thresh").as_double();
    linear_thresh_   = this->get_parameter("linear_thresh").as_double();
    cmd_timeout_ms_  = this->get_parameter("cmd_timeout_ms").as_int();

    RCLCPP_INFO(this->get_logger(),
      "serial_bridge  port=%s  baud=%d  fixed_pwm=%d",
      port.c_str(), baud, fixed_pwm_);

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

    // ── /cmd_vel subscriber ─────────────────────────────────────
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        handle_cmd_vel(msg);
      });

    // ── Watchdog — zeros motors if /cmd_vel goes silent ─────────
    watchdog_timer_ = this->create_wall_timer(
      50ms,
      [this]() { check_cmd_timeout(); });

    // ── Background thread — reads encoder lines from Arduino ────
    read_thread_ = std::thread([this]() { read_loop(); });

    RCLCPP_INFO(this->get_logger(), "serial_bridge ready");
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
  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    double linear  = msg->linear.x;
    double angular = msg->angular.z;

    int left_pwm  = 0;
    int right_pwm = 0;

    if (std::abs(angular) >= angular_thresh_) {
      // ── Rotate in place — angular always wins ─────────────────
      // angular.z > 0 = CCW / turn left:
      //   left wheel backward, right wheel forward
      // angular.z < 0 = CW / turn right:
      //   left wheel forward, right wheel backward
      int sign  = (angular > 0.0) ? 1 : -1;
      left_pwm  = -sign * fixed_pwm_;
      right_pwm =  sign * fixed_pwm_;

    } else if (std::abs(linear) >= linear_thresh_) {
      // ── Drive straight ────────────────────────────────────────
      int sign  = (linear > 0.0) ? 1 : -1;
      left_pwm  = sign * fixed_pwm_;
      right_pwm = sign * fixed_pwm_;
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
        if (buf.size() > 128) buf.clear();  // guard against garbage
      }
    }
  }

  // ── Parse "E:{left},{right}" from Arduino ─────────────────────
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

      int32_t left_ticks  = std::stoi(line.substr(2, comma - 2));
      int32_t right_ticks = std::stoi(line.substr(comma + 1));

      auto lmsg = std_msgs::msg::Int32();
      auto rmsg = std_msgs::msg::Int32();
      lmsg.data = left_ticks;
      rmsg.data = right_ticks;

      left_ticks_pub_->publish(lmsg);
      right_ticks_pub_->publish(rmsg);
    }
    catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Bad encoder line [%s]: %s",
                  line.c_str(), e.what());
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_ticks_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::atomic<bool>  running_;
  std::thread        read_thread_;
  int                serial_fd_;
  rclcpp::Time       last_cmd_time_;

  int    fixed_pwm_;
  double angular_thresh_;
  double linear_thresh_;
  int    cmd_timeout_ms_;
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