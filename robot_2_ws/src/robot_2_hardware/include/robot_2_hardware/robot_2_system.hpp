#ifndef ROBOT_2_HARDWARE__ROBOT_2_SYSTEM_HPP_
#define ROBOT_2_HARDWARE__ROBOT_2_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_2_hardware
{

class Robot2System : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(Robot2System)

  // ==========================================================
  // ROS 2 CONTROL LIFECYCLE
  // ==========================================================

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;


  // ==========================================================
  // INTERFACES
  // ==========================================================

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;


  // ==========================================================
  // HARDWARE COMMUNICATION
  // ==========================================================

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;


private:

  // ==========================================================
  // SERIAL
  // ==========================================================

  bool openSerial();

  void closeSerial();

  bool configureSerial();

  bool writeSerial(
    const std::string & command);

  bool readLine(
    std::string & line,
    int timeout_ms);


  // ==========================================================
  // ARDUINO PROTOCOL
  //
  // Firmware line formats (ZingiraFirmware, wheel order is
  // always FL FR BL BR, matching joint_names_ index order):
  //
  //   request:  "e\n"
  //   response: "t_fl t_fr t_bl t_br\n"
  //
  //   command:  "m v_fl v_fr v_bl v_br\n"
  //
  // Unlike the Uno firmware, each of the 4 wheels now has its
  // own encoder and its own motor channel - there is no more
  // front/rear averaging on either the read or write path.
  // ==========================================================

  static constexpr std::size_t kNumWheels = 4;

  bool requestEncoders(
    std::array<long, kNumWheels> & ticks);

  bool sendMotorCommand(
    const std::array<double, kNumWheels> & ticks_per_period);


  // ==========================================================
  // CONVERSIONS
  // ==========================================================

  double velocityToTicksPerPeriod(
    double velocity) const;

  double ticksToRadians(
    double ticks) const;

  double ticksPerSecondToRadiansPerSecond(
    long delta_ticks,
    double period_seconds) const;


  // ==========================================================
  // PARAMETER HELPERS
  // ==========================================================

  double getNumericParameter(
    const std::string & name,
    double default_value) const;

  std::string getStringParameter(
    const std::string & name,
    const std::string & default_value) const;


  // ==========================================================
  // JOINT INFORMATION
  //
  // Order (fixed, validated in on_init):
  //
  // 0 = front_left  (FL)
  // 1 = front_right (FR)
  // 2 = back_left   (BL)
  // 3 = back_right  (BR)
  // ==========================================================

  std::vector<std::string> joint_names_;


  // ==========================================================
  // ROS 2 CONTROL STORAGE
  // ==========================================================

  std::vector<double> hw_commands_;

  std::vector<double> hw_positions_;

  std::vector<double> hw_velocities_;


  // ==========================================================
  // SERIAL CONFIGURATION
  // ==========================================================

  std::string device_;

  int baud_rate_{57600};

  int timeout_ms_{200};

  int serial_fd_{-1};


  // ==========================================================
  // ROBOT PARAMETERS
  // ==========================================================

  double encoder_ticks_per_rev_{1980.0};

  double pid_period_seconds_{0.05};

  double wheel_radius_{0.0425};

  double wheel_separation_{0.30};


  // ==========================================================
  // PER-WHEEL DIRECTION CORRECTIONS
  //
  // Indexed FL, FR, BL, BR. Each wheel now has its own motor
  // and encoder, so each can independently be wired backwards -
  // a single left/right sign pair (as on the Uno build) is no
  // longer enough. Parameter names in ros2_control.xacro:
  //   front_left_encoder_sign / front_left_command_sign
  //   front_right_encoder_sign / front_right_command_sign
  //   back_left_encoder_sign / back_left_command_sign
  //   back_right_encoder_sign / back_right_command_sign
  // ==========================================================

  std::array<double, kNumWheels> encoder_sign_{1.0, 1.0, 1.0, 1.0};

  std::array<double, kNumWheels> command_sign_{1.0, 1.0, 1.0, 1.0};


  // ==========================================================
  // ENCODER STATE
  // ==========================================================

  std::array<long, kNumWheels> previous_ticks_{0, 0, 0, 0};

  bool first_read_{true};


  // ==========================================================
  // HARDWARE STATE
  // ==========================================================

  bool active_{false};
};

}  // namespace robot_2_hardware

#endif  // ROBOT_2_HARDWARE__ROBOT_2_SYSTEM_HPP_