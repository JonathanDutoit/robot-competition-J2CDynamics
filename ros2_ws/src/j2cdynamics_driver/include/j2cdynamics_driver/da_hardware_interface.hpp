#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "j2cdynamics_driver/common_hardware_interface.hpp"

namespace j2cdynamics_driver
{

enum class SweeperMode : int
{
  Idle = 0,
  Collect = 1,
  Dropoff = 2,
  Fault = 3
};

class DaHardwareInterface : public CommonHardwareInterface
{
public:
  virtual hardware_interface::CallbackReturn extra_joint_init_sanity_check(
    const hardware_interface::HardwareInfo & info) override;

  virtual hardware_interface::return_type read_extra(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  virtual hardware_interface::return_type write_extra(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  virtual void export_extra_state_interfaces(std::vector<hardware_interface::StateInterface> & interfaces) override;
  virtual void export_extra_command_interfaces(std::vector<hardware_interface::CommandInterface> & interfaces) override;

private:
  bool request_mode(SweeperMode & mode);
  
  double hw_cmd_mode_{0.0};
  double hw_mode_state_{0.0};

  int sweeper_joint_idx_{-1};

  // Internal sweeper state tracking
  SweeperMode current_mode_ {SweeperMode::Idle};
  SweeperMode last_send_mode_    {SweeperMode::Idle};

  SweeperMode decode_mode(double v) const;
  std::string encode_mode_command(SweeperMode mode) const;
  bool parse_mode(const std::string & line, SweeperMode & mode);
};

}