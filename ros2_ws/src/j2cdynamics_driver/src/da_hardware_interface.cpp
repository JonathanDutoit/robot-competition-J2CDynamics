#include "j2cdynamics_driver/da_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>      
#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace j2cdynamics_driver
{

hardware_interface::CallbackReturn DaHardwareInterface::extra_joint_init_sanity_check(
  const hardware_interface::HardwareInfo & info)
{
  /// ── reset indices ───────────────────────────
  sweeper_joint_idx_ = -1;

  for (size_t i = 0; i < info.joints.size(); ++i)
  {
    const std::string & name = info.joints[i].name;

    if (name.find("sweeper") != std::string::npos)
      sweeper_joint_idx_ = static_cast<int>(i);
  }

  // ── required joints check ──────

  if (sweeper_joint_idx_ < 0)
  {
    RCLCPP_FATAL(logger_,
      "Missing required sweeper joint.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_,
    "Robot specific Hardware initialized: sweeper=%d",
    sweeper_joint_idx_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DaHardwareInterface::export_extra_state_interfaces(
    std::vector<hardware_interface::StateInterface> & interfaces
) {
    interfaces.emplace_back(
        info_.joints[sweeper_joint_idx_].name,
        hardware_interface::HW_IF_POSITION,
        &hw_mode_state_);
}

void DaHardwareInterface::export_extra_command_interfaces(
    std::vector<hardware_interface::CommandInterface> & interfaces
) {
    interfaces.emplace_back(
        info_.joints[sweeper_joint_idx_].name,
        hardware_interface::HW_IF_POSITION,
        &hw_cmd_mode_);
}

hardware_interface::return_type DaHardwareInterface::read_extra(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // ── MODE POLLING ─────────────────────
  SweeperMode mode;

  if (request_mode(mode)) {
    hw_mode_state_ = static_cast<double>(mode);
    if (hw_cmd_mode_ != hw_mode_state_) {
      RCLCPP_WARN(logger_,
        "Mode mismatch: commanded=%d actual=%d",
        (int)hw_cmd_mode_,
        (int)hw_mode_state_);
    }
    return hardware_interface::return_type::OK;
  } else {
    // On a read failure, keep hw_mode_state_ unchanged (last good value) and
    // let the failure counter govern escalation rather than faulting on a single
    // hiccup. Note that the mode state is informational only (not used in
    // control logic), so it's less critical to have it update every cycle.
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_,
        "Arduino unresponsive: %d consecutive failed reads — reporting fault",
        consecutive_failures_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Mode read failed (%d in a row) — reusing last value",
      consecutive_failures_);
    return hardware_interface::return_type::OK;
  }
}

hardware_interface::return_type DaHardwareInterface::write_extra(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
  SweeperMode desired_mode = decode_mode(hw_cmd_mode_);

  if (desired_mode != last_send_mode_) {
    std::string cmd = encode_mode_command(desired_mode);

    if (!send_command(cmd)) {
      ++consecutive_failures_;
      if (consecutive_failures_ > failure_threshold_) {
        RCLCPP_ERROR(logger_, "Repeated MODE write failures — reporting fault");
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
        "Failed to send MODE command (%d in a row)", consecutive_failures_);
      return hardware_interface::return_type::ERROR;
    } else {
      last_send_mode_ = desired_mode;
      return hardware_interface::return_type::OK;
    }
  }
  return hardware_interface::return_type::OK;
}

bool DaHardwareInterface::request_mode(SweeperMode & mode)
{
  try {
    // NOTE: flushing before each request is acceptable in a strict
    // request/response protocol (clears stale partial replies). If you ever
    // move the Arduino to streaming mode, drop this and the Write below.
    serial_.FlushInputBuffer();
    serial_.Write("MODE\n");
    std::string line;

    // Short timeout: a late Arduino must not stall the control loop for long.
    // (Was 50 with a comment claiming 500 — reconcile this with your odom_rate.)
    serial_.ReadLine(line, '\n', 50);

    return parse_mode(line, mode);

  } catch (const LibSerial::ReadTimeout &) {
    // No reply this cycle — a normal transient, not an exception worth logging
    // at error level. Caller treats the false return as "reuse last value".
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial read error: %s", e.what());
    return false;
  }
}

// Accept the MODE line even with leading noise/whitespace, rather than
// demanding it start exactly at index 0. Still strict about the numeric format.
bool DaHardwareInterface::parse_mode(
  const std::string & line, SweeperMode & mode)
{
  std::size_t start = line.find("MODE:");
  if (start == std::string::npos) return false;

  std::string value = line.substr(start);

  if (value.find("IDLE") != std::string::npos) {
    mode = SweeperMode::Idle;
    return true;
  }
  if (value.find("COLLECT") != std::string::npos) {
    mode = SweeperMode::Collect;
    return true;
  }
  if (value.find("DROPOFF") != std::string::npos) {
    mode = SweeperMode::Dropoff;
    return true;
  }

  return false;
}

SweeperMode DaHardwareInterface::decode_mode(double v) const
{
  int mode_int = static_cast<int>(std::round(v));
  switch (mode_int) {
    case 0: return SweeperMode::Idle;
    case 1: return SweeperMode::Collect;
    case 2: return SweeperMode::Dropoff;
    case 3: return SweeperMode::Fault;
    default:
      RCLCPP_WARN(logger_, "Invalid mode command value: %.2f — treating as Idle", v);
      return SweeperMode::Idle;
  }
}

std::string DaHardwareInterface::encode_mode_command(SweeperMode mode) const
{
  switch (mode)
  {
    case SweeperMode::Idle:
      return "IDLE\n";

    case SweeperMode::Collect:
      return "COLLECT\n";

    case SweeperMode::Dropoff:
      return "DROPOFF\n";
    default:
      RCLCPP_WARN(logger_, "Attempting to command invalid mode: %d — treating as Idle", (int)mode);
      return "IDLE\n";
  }
}

} // namespace j2cdynamics_driver

// ── Plugin registration ───────────────────────────────────────────────────────
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  j2cdynamics_driver::DaHardwareInterface,
  hardware_interface::SystemInterface)