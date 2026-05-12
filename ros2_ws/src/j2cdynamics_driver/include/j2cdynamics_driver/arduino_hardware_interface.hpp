#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <libserial/SerialPort.h>

namespace j2cdynamics_driver
{

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ros2_control interface
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial helpers
  bool send_command(const std::string & cmd);
  bool request_odometry(double & left_vel, double & right_vel);

  // Config (loaded from URDF <hardware> params)
  std::string port_;
  int         baudrate_;
  double      max_rad_s_;
  double      ramp_step_;

  // Serial port
  LibSerial::SerialPort serial_;

  // State interfaces — what ros2_control reads FROM hardware
  double hw_vel_left_  {0.0};
  double hw_vel_right_ {0.0};
  double hw_pos_left_  {0.0};   // integrated position (rad)
  double hw_pos_right_ {0.0};

  // Command interfaces — what ros2_control writes TO hardware
  double hw_cmd_left_  {0.0};
  double hw_cmd_right_ {0.0};

  // Ramping (mirrors your Python ArduinoBridge logic)
  double current_left_  {0.0};
  double current_right_ {0.0};

  double _ramp(double current, double target) const;
  double _clamp(double value)                 const;

  rclcpp::Logger logger_ {rclcpp::get_logger("ArduinoHardwareInterface")};
};

} 