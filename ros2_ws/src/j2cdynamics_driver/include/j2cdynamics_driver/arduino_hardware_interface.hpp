#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "rclcpp_lifecycle/state.hpp"

#include <libserial/SerialPort.h>

namespace j2cdynamics_driver
{

enum class SweeperMode : int
{
  Idle = 0,
  Collect = 1,
  Dropoff = 2,
  Fault = 3
};

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
  bool request_mode(SweeperMode & mode);
  bool request_duplo_count(double & count);

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
  double hw_mode_state_{0.0};
  double hw_duplo_count_{0.0};

  // Command interfaces — what ros2_control writes TO hardware
  double hw_cmd_left_  {0.0};
  double hw_cmd_right_ {0.0};
  double hw_cmd_mode_{0.0};
  double hw_cmd_duplo_count_{0.0};

  // resolved joint indices (set in on_init)
  int left_joint_idx_{-1};
  int right_joint_idx_{-1};
  int sweeper_joint_idx_{-1};
  int duplo_counter_joint_idx_{-1};

  // fault escalation
  int consecutive_failures_{0};
  int failure_threshold_{10};   // tune to your odom_rate (e.g. ~0.2s at 50 Hz)

  // Ramping (mirrors your Python ArduinoBridge logic)
  double current_left_  {0.0};
  double current_right_ {0.0};

  // Internal sweeper state tracking
  SweeperMode current_mode_ {SweeperMode::Idle};
  SweeperMode last_send_mode_    {SweeperMode::Idle};

  // Helper functions
  double _ramp(double current, double target) const;
  double _clamp(double value)                 const;
  SweeperMode decode_mode(double v) const;
  std::string encode_mode_command(SweeperMode mode) const;
  bool decode_duplo_count_request(double v) const;

  // Parsing helpers
  bool parse_odometry(const std::string & line, double & left_vel, double & right_vel);
  bool parse_mode(const std::string & line, SweeperMode & mode);
  bool parse_duplo_count(const std::string & line, double & count);

  // dt-spike warn throttle
  std::chrono::steady_clock::time_point last_dt_warn_{};

  rclcpp::Clock clock_{RCL_STEADY_TIME};

  rclcpp::Logger logger_ {rclcpp::get_logger("ArduinoHardwareInterface")};
};

} 