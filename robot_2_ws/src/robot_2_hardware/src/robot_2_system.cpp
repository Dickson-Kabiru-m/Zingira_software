#include "robot_2_hardware/robot_2_system.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

namespace robot_2_hardware
{

// ============================================================
// Constructor / Initialization
// ============================================================

hardware_interface::CallbackReturn Robot2System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ----------------------------------------------------------
  // Validate number of joints
  // ----------------------------------------------------------

  if (info_.joints.size() != kNumWheels) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Expected exactly %zu wheel joints, got %zu",
      kNumWheels,
      info_.joints.size());

    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();

  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);

    // --------------------------------------------------------
    // Each joint must have one velocity command interface
    // --------------------------------------------------------

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("robot_2_hardware"),
        "Joint '%s' must have exactly one velocity command interface",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    // --------------------------------------------------------
    // Position + velocity state interfaces
    // --------------------------------------------------------

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("robot_2_hardware"),
        "Joint '%s' must have position and velocity state interfaces",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position = false;
    bool has_velocity = false;

    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      }

      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity = true;
      }
    }

    if (!has_position || !has_velocity) {
      RCLCPP_ERROR(
        rclcpp::get_logger("robot_2_hardware"),
        "Joint '%s' must provide position and velocity state interfaces",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ----------------------------------------------------------
  // Allocate storage
  // ----------------------------------------------------------

  hw_commands_.assign(kNumWheels, 0.0);
  hw_positions_.assign(kNumWheels, 0.0);
  hw_velocities_.assign(kNumWheels, 0.0);

  // ----------------------------------------------------------
  // Hardware parameters
  // ----------------------------------------------------------

  device_ = getStringParameter(
    "device",
    "/dev/ttyACM0");

  baud_rate_ = static_cast<int>(
    getNumericParameter(
      "baud_rate",
      57600));

  timeout_ms_ = static_cast<int>(
    getNumericParameter(
      "timeout_ms",
      200));

  encoder_ticks_per_rev_ =
    getNumericParameter(
      "encoder_ticks_per_rev",
      1980.0);

  pid_period_seconds_ =
    getNumericParameter(
      "pid_period",
      0.05);

  wheel_radius_ =
    getNumericParameter(
      "wheel_radius",
      0.0425);

  wheel_separation_ =
    getNumericParameter(
      "wheel_separation",
      0.30);

  // ----------------------------------------------------------
  // Per-wheel direction correction parameters.
  // Index order matches joint_names_: 0=FL, 1=FR, 2=BL, 3=BR.
  // ----------------------------------------------------------

  encoder_sign_[0] = getNumericParameter("front_left_encoder_sign", 1.0);
  encoder_sign_[1] = getNumericParameter("front_right_encoder_sign", 1.0);
  encoder_sign_[2] = getNumericParameter("back_left_encoder_sign", 1.0);
  encoder_sign_[3] = getNumericParameter("back_right_encoder_sign", 1.0);

  command_sign_[0] = getNumericParameter("front_left_command_sign", 1.0);
  command_sign_[1] = getNumericParameter("front_right_command_sign", 1.0);
  command_sign_[2] = getNumericParameter("back_left_command_sign", 1.0);
  command_sign_[3] = getNumericParameter("back_right_command_sign", 1.0);

  // ----------------------------------------------------------
  // Initial state
  // ----------------------------------------------------------

  previous_ticks_.fill(0);

  first_read_ = true;
  active_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Robot 2 hardware interface initialized (4 independent wheels)");

  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Serial device: %s",
    device_.c_str());

  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Baud rate: %d",
    baud_rate_);

  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Encoder ticks/rev: %.2f",
    encoder_ticks_per_rev_);

  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Wheel radius: %.4f m",
    wheel_radius_);

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// Configure
// ============================================================

hardware_interface::CallbackReturn Robot2System::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Configuring Robot 2 hardware");

  if (!openSerial()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Failed to open serial device %s",
      device_.c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset Arduino encoder counters
  if (!writeSerial("r\n")) {
    RCLCPP_WARN(
      rclcpp::get_logger("robot_2_hardware"),
      "Could not send encoder reset command");
  }

  previous_ticks_.fill(0);
  first_read_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// Activate
// ============================================================

hardware_interface::CallbackReturn Robot2System::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Activating Robot 2 hardware");

  // Ensure motors are stopped before operation
  if (!writeSerial("s\n")) {
    RCLCPP_WARN(
      rclcpp::get_logger("robot_2_hardware"),
      "Failed to send stop command during activation");
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  previous_ticks_.fill(0);
  first_read_ = true;

  active_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// Deactivate
// ============================================================

hardware_interface::CallbackReturn Robot2System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Deactivating Robot 2 hardware");

  writeSerial("s\n");

  active_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// Export state interfaces
// ============================================================

std::vector<hardware_interface::StateInterface>
Robot2System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}


// ============================================================
// Export command interfaces
// ============================================================

std::vector<hardware_interface::CommandInterface>
Robot2System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {

    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_[i]);
  }

  return command_interfaces;
}


// ============================================================
// READ
//
// Arduino command:  "e\n"
// Arduino response: "t_fl t_fr t_bl t_br\n"
//
// Each wheel now has its own encoder, so positions/velocities
// are computed independently per wheel - no more duplicating
// one side's reading across its front and back joint.
// ============================================================

hardware_interface::return_type Robot2System::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<long, kNumWheels> ticks{};

  if (!requestEncoders(ticks)) {

    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Failed to read encoder values");

    return hardware_interface::return_type::ERROR;
  }

  const double period_seconds = period.seconds();

  for (std::size_t i = 0; i < kNumWheels; ++i) {

    const double signed_ticks =
      static_cast<double>(ticks[i]) * encoder_sign_[i];

    hw_positions_[i] = ticksToRadians(signed_ticks);

    if (first_read_) {

      hw_velocities_[i] = 0.0;

    } else if (period_seconds > 0.0) {

      const long delta =
        static_cast<long>(
          (ticks[i] - previous_ticks_[i]) * encoder_sign_[i]);

      hw_velocities_[i] =
        ticksPerSecondToRadiansPerSecond(delta, period_seconds);
    }

    previous_ticks_[i] = ticks[i];
  }

  first_read_ = false;

  return hardware_interface::return_type::OK;
}


// ============================================================
// WRITE
//
// ros2_control gives wheel angular velocities in rad/s, one per
// independent wheel. No more averaging front/rear per side -
// each wheel gets its own command sign correction and its own
// ticks-per-period value sent straight to the firmware.
//
// rad/s -> rad/PID-period -> rev/PID-period -> ticks/PID-period
// ============================================================

hardware_interface::return_type Robot2System::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, kNumWheels> ticks_per_period{};

  for (std::size_t i = 0; i < kNumWheels; ++i) {

    const double corrected = hw_commands_[i] * command_sign_[i];
    ticks_per_period[i] = velocityToTicksPerPeriod(corrected);
  }

  if (!sendMotorCommand(ticks_per_period)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Failed to send motor command");

    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


// ============================================================
// SERIAL OPEN
// ============================================================

bool Robot2System::openSerial()
{
  if (serial_fd_ >= 0) {
    return true;
  }

  serial_fd_ = ::open(
    device_.c_str(),
    O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd_ < 0) {

    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Unable to open %s: %s",
      device_.c_str(),
      std::strerror(errno));

    return false;
  }

  if (!configureSerial()) {

    closeSerial();

    return false;
  }

  // Give Arduino time to reset after opening serial
  usleep(2000000);

  // Flush stale data
  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(
    rclcpp::get_logger("robot_2_hardware"),
    "Serial connection established");

  return true;
}


// ============================================================
// SERIAL CLOSE
// ============================================================

void Robot2System::closeSerial()
{
  if (serial_fd_ >= 0) {

    ::close(serial_fd_);

    serial_fd_ = -1;
  }
}


// ============================================================
// SERIAL CONFIGURATION
// ============================================================

bool Robot2System::configureSerial()
{
  struct termios tty {};

  if (tcgetattr(serial_fd_, &tty) != 0) {

    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "tcgetattr failed: %s",
      std::strerror(errno));

    return false;
  }

  cfmakeraw(&tty);

  speed_t speed;

  switch (baud_rate_) {

    case 9600:
      speed = B9600;
      break;

    case 19200:
      speed = B19200;
      break;

    case 38400:
      speed = B38400;
      break;

    case 57600:
      speed = B57600;
      break;

    case 115200:
      speed = B115200;
      break;

    default:

      RCLCPP_ERROR(
        rclcpp::get_logger("robot_2_hardware"),
        "Unsupported baud rate: %d",
        baud_rate_);

      return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {

    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "tcsetattr failed: %s",
      std::strerror(errno));

    return false;
  }

  return true;
}


// ============================================================
// SERIAL WRITE
// ============================================================

bool Robot2System::writeSerial(
  const std::string & command)
{
  if (serial_fd_ < 0) {
    return false;
  }

  const char * data = command.c_str();
  std::size_t remaining = command.size();

  while (remaining > 0) {

    const ssize_t result = ::write(serial_fd_, data, remaining);

    if (result < 0) {

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }

      RCLCPP_ERROR(
        rclcpp::get_logger("robot_2_hardware"),
        "Serial write failed: %s",
        std::strerror(errno));

      return false;
    }

    data += result;
    remaining -= static_cast<std::size_t>(result);
  }

  return true;
}


// ============================================================
// READ LINE
// ============================================================

bool Robot2System::readLine(
  std::string & line,
  int timeout_ms)
{
  line.clear();

  if (serial_fd_ < 0) {
    return false;
  }

  const auto start = std::chrono::steady_clock::now();

  while (true) {

    const auto now = std::chrono::steady_clock::now();

    const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        now - start).count();

    if (elapsed >= timeout_ms) {
      return false;
    }

    const int remaining_timeout = timeout_ms - static_cast<int>(elapsed);

    struct pollfd pfd {};

    pfd.fd = serial_fd_;
    pfd.events = POLLIN;

    const int result = poll(&pfd, 1, remaining_timeout);

    if (result < 0) {

      if (errno == EINTR) {
        continue;
      }

      return false;
    }

    if (result == 0) {
      return false;
    }

    if (pfd.revents & POLLIN) {

      char buffer[64];

      const ssize_t bytes = ::read(serial_fd_, buffer, sizeof(buffer));

      if (bytes <= 0) {
        continue;
      }

      for (ssize_t i = 0; i < bytes; ++i) {

        const char c = buffer[i];

        if (c == '\n' || c == '\r') {

          if (!line.empty()) {
            return true;
          }

        } else {

          line += c;

          if (line.size() >= 256) {
            return false;
          }
        }
      }
    }
  }
}


// ============================================================
// REQUEST ENCODERS
//
// Expects a response line: "t_fl t_fr t_bl t_br"
// ============================================================

bool Robot2System::requestEncoders(
  std::array<long, kNumWheels> & ticks)
{
  if (!writeSerial("e\n")) {

    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Failed to request encoder data");

    return false;
  }

  std::string line;

  if (!readLine(line, timeout_ms_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Timed out waiting for encoder data");

    return false;
  }

  std::stringstream ss(line);

  if (!(ss >> ticks[0] >> ticks[1] >> ticks[2] >> ticks[3])) {

    RCLCPP_ERROR(
      rclcpp::get_logger("robot_2_hardware"),
      "Invalid encoder response: '%s'",
      line.c_str());

    return false;
  }

  return true;
}


// ============================================================
// SEND MOTOR COMMAND
//
// Arduino expects: "m t_fl t_fr t_bl t_br\n"
// ============================================================

bool Robot2System::sendMotorCommand(
  const std::array<double, kNumWheels> & ticks_per_period)
{
  std::ostringstream command;

  command.setf(std::ios::fixed);
  command.precision(4);

  command << "m";

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    command << " " << ticks_per_period[i];
  }

  command << "\n";

  return writeSerial(command.str());
}


// ============================================================
// VELOCITY -> TICKS / PID PERIOD
// ============================================================

double Robot2System::velocityToTicksPerPeriod(
  double velocity) const
{
  const double radians_per_period = velocity * pid_period_seconds_;
  const double revolutions_per_period = radians_per_period / (2.0 * M_PI);

  return revolutions_per_period * encoder_ticks_per_rev_;
}


// ============================================================
// TICKS -> RADIANS
// ============================================================

double Robot2System::ticksToRadians(
  double ticks) const
{
  return ticks * (2.0 * M_PI) / encoder_ticks_per_rev_;
}


// ============================================================
// TICKS / SECOND -> RAD / SECOND
// ============================================================

double Robot2System::ticksPerSecondToRadiansPerSecond(
  long delta_ticks,
  double period_seconds) const
{
  if (period_seconds <= 0.0) {
    return 0.0;
  }

  const double ticks_per_second =
    static_cast<double>(delta_ticks) / period_seconds;

  return ticks_per_second * (2.0 * M_PI) / encoder_ticks_per_rev_;
}


// ============================================================
// NUMERIC PARAMETER
// ============================================================

double Robot2System::getNumericParameter(
  const std::string & name,
  double default_value) const
{
  const auto it = info_.hardware_parameters.find(name);

  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  try {

    return std::stod(it->second);

  } catch (...) {

    RCLCPP_WARN(
      rclcpp::get_logger("robot_2_hardware"),
      "Invalid numeric parameter '%s': '%s'. Using default %.3f",
      name.c_str(),
      it->second.c_str(),
      default_value);

    return default_value;
  }
}


// ============================================================
// STRING PARAMETER
// ============================================================

std::string Robot2System::getStringParameter(
  const std::string & name,
  const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(name);

  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  return it->second;
}

}  // namespace robot_2_hardware


// ============================================================
// PLUGIN EXPORT
// ============================================================

PLUGINLIB_EXPORT_CLASS(
  robot_2_hardware::Robot2System,
  hardware_interface::SystemInterface)