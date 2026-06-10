#include "j2cdynamics_driver/common_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>      
#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace j2cdynamics_driver
{

// ── Lifecycle: on_init ────────────────────────────────────────────────────────
// Reads parameters from the <ros2_control> block in your URDF
hardware_interface::CallbackReturn
CommonHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read params declared in the URDF <hardware> block
  port_      = info_.hardware_parameters.at("port");
  baudrate_  = std::stoi(info_.hardware_parameters.at("baudrate"));
  max_rad_s_ = std::stod(info_.hardware_parameters.at("max_rad_s"));
  ramp_step_ = std::stod(info_.hardware_parameters.at("ramp_step"));

  joint_init_sanity_check(info);
  extra_joint_init_sanity_check(info);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_configure ───────────────────────────────────────────────────
// Open the serial port here (not in on_init) so it can be retried on failure
hardware_interface::CallbackReturn
CommonHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    serial_.Open(port_);
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);   // match your Arduino
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  } catch (const LibSerial::OpenFailed &) {
    RCLCPP_FATAL(logger_, "Failed to open serial port: %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Waiting for Arduino to boot...");
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  serial_.FlushIOBuffers();
  RCLCPP_INFO(logger_, "Arduino ready");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_activate / on_deactivate / on_cleanup ──────────────────────
hardware_interface::CallbackReturn
CommonHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  hw_cmd_left_ = hw_cmd_right_ = 0.0;
  current_left_ = current_right_ = 0.0;
  hw_vel_left_ = hw_vel_right_ = 0.0;
  hw_duplo_count_ = 0.0;
  consecutive_failures_ = 0;
  RCLCPP_INFO(logger_, "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
CommonHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Send zero velocity before deactivating
  send_command("SPEED 0.0000 0.0000\n");
  send_command("MODE IDLE\n");
  RCLCPP_INFO(logger_, "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
CommonHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (serial_.IsOpen()) serial_.Close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Export interfaces ─────────────────────────────────────────────────────────
// Now keyed off the resolved indices so the exported names always match the
// physical wheel the hw_* variable represents.
std::vector<hardware_interface::StateInterface>
CommonHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[left_joint_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_vel_left_);
  state_interfaces.emplace_back(
    info_.joints[left_joint_idx_].name,  hardware_interface::HW_IF_POSITION, &hw_pos_left_);
  state_interfaces.emplace_back(
    info_.joints[right_joint_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_right_);
  state_interfaces.emplace_back(
    info_.joints[right_joint_idx_].name, hardware_interface::HW_IF_POSITION, &hw_pos_right_);
  state_interfaces.emplace_back(
    info_.joints[duplo_counter_joint_idx_].name, hardware_interface::HW_IF_POSITION, &hw_duplo_count_);

  // Append any robot-specific state interfaces from the derived class
  export_extra_state_interfaces(state_interfaces);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CommonHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[left_joint_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_cmd_left_);
  command_interfaces.emplace_back(
    info_.joints[right_joint_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_right_);
  command_interfaces.emplace_back(
    info_.joints[duplo_counter_joint_idx_].name, hardware_interface::HW_IF_POSITION, &hw_cmd_duplo_count_);

  // Append any robot-specific command interfaces from the derived class
  export_extra_command_interfaces(command_interfaces);

  return command_interfaces;
}

// ── read() — poll Arduino for wheel velocities ────────────────────────────────
hardware_interface::return_type
CommonHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // ── ODOMETRY POLLING ─────────────────────
  double left_vel = 0.0, right_vel = 0.0;

  if (request_odometry(left_vel, right_vel)) {
    // ── success: update state + reset failure counter ──
    hw_vel_left_  = left_vel;
    hw_vel_right_ = right_vel;
    consecutive_failures_ = 0;
    
    //RCLCPP_INFO(logger_, "Left wheel=%f - Right wheel=%f", hw_vel_left_, hw_vel_right_);
  } else {
    // Returning ERROR here makes ros2_control deactivate the component. Instead
    // we keep the last good velocity (hw_vel_* already hold it) and only escalate
    // to a real fault after a sustained run of failures.
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_,
        "Arduino unresponsive: %d consecutive failed reads — reporting fault",
        consecutive_failures_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Odometry read failed (%d in a row) — reusing last value",
      consecutive_failures_);
  }

  hw_pos_left_  += hw_vel_left_  * period.seconds();
  hw_pos_right_ += hw_vel_right_ * period.seconds();

  // ── DUPLO COUNT POLLING ─────────────────────
  double duplo_count;

  if (request_duplo_count(duplo_count)) {
    hw_duplo_count_ = duplo_count;
  } else {
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_,
        "Arduino unresponsive: %d consecutive failed reads — reporting fault",
        consecutive_failures_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Duplo count read failed (%d in a row) — reusing last value",
      consecutive_failures_);
  }

   if (read_extra(time, period) == hardware_interface::return_type::ERROR) {
     return hardware_interface::return_type::ERROR;
   }

  return hardware_interface::return_type::OK;
}

// ── write() — send ramped velocity command to Arduino ────────────────────────
hardware_interface::return_type
CommonHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double left  = _ramp(current_left_,  _clamp(hw_cmd_left_));
  double right = _ramp(current_right_, _clamp(hw_cmd_right_));

  current_left_  = left;
  current_right_ = right;

  std::ostringstream cmd;
  cmd << "SPEED " << std::fixed << std::setprecision(4) << left
      << " " << right << "\n";

  if (!send_command(cmd.str())) {
    // A failed write is more serious than a failed read (we can't reuse a
    // "last command"), but still let the failure counter govern escalation
    // rather than faulting on a single hiccup.
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_, "Repeated SPEED write failures — reporting fault");
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Failed to send SPEED command (%d in a row)", consecutive_failures_);
  }

  bool duplo_count_requested = decode_duplo_count_request(hw_cmd_duplo_count_);

  if (duplo_count_requested) {
    if (!send_command("DUPLO_COUNT\n")) {
      ++consecutive_failures_;
      if (consecutive_failures_ > failure_threshold_) {
        RCLCPP_ERROR(logger_, "Repeated DUPLO_COUNT write failures — reporting fault");
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
        "Failed to send DUPLO_COUNT command (%d in a row)", consecutive_failures_);
    }
  }

  if (write_extra(time, period) == hardware_interface::return_type::ERROR) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ── Private helpers ───────────────────────────────────────────────────────────
hardware_interface::CallbackReturn CommonHardwareInterface::joint_init_sanity_check(
  const hardware_interface::HardwareInfo & info)
{
  /// ── reset indices ───────────────────────────
  left_joint_idx_ = -1;
  right_joint_idx_ = -1;
  duplo_counter_joint_idx_ = -1;

  // ── detect joints (robust, optional duplo) ──
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const std::string & name = info_.joints[i].name;

    if (name.find("left") != std::string::npos)
      left_joint_idx_ = static_cast<int>(i);

    else if (name.find("right") != std::string::npos)
      right_joint_idx_ = static_cast<int>(i);

    else if (name.find("duplo") != std::string::npos)
      duplo_counter_joint_idx_ = static_cast<int>(i);
  }

  // ── required joints check (core robot) ──────
  if (left_joint_idx_ < 0 || right_joint_idx_ < 0)
  {
    RCLCPP_FATAL(logger_,
      "Missing required wheel joints (left/right).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (duplo_counter_joint_idx_ < 0)
  {
    RCLCPP_WARN(logger_,
      "Duplo counter not found → running in reduced capability mode");
  }

  RCLCPP_INFO(logger_,
    "Hardware initialized: left=%d right=%d duplo=%d",
    left_joint_idx_,
    right_joint_idx_,
    duplo_counter_joint_idx_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool CommonHardwareInterface::send_command(const std::string & cmd)
{
  try {
    serial_.Write(cmd);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial write error: %s", e.what());
    return false;
  }
}

bool CommonHardwareInterface::request_odometry(double & left_vel, double & right_vel)
{
  try {
    // NOTE: flushing before each request is acceptable in a strict
    // request/response protocol (clears stale partial replies). If you ever
    // move the Arduino to streaming mode, drop this and the Write below.
    serial_.FlushInputBuffer();
    serial_.Write("ODOMETRY\n");
    std::string line;

    // Short timeout: a late Arduino must not stall the control loop for long.
    // (Was 50 with a comment claiming 500 — reconcile this with your odom_rate.)
    serial_.ReadLine(line, '\n', 50);

    return parse_odometry(line, left_vel, right_vel);

  } catch (const LibSerial::ReadTimeout &) {
    // No reply this cycle — a normal transient, not an exception worth logging
    // at error level. Caller treats the false return as "reuse last value".
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial read error: %s", e.what());
    return false;
  }
}

// Accept the ODOMETRY line even with leading noise/whitespace, rather than
// demanding it start exactly at index 0. Still strict about the numeric format.
bool CommonHardwareInterface::parse_odometry(
  const std::string & line, double & left_vel, double & right_vel)
{
  std::size_t start = line.find("ODOMETRY");
  if (start == std::string::npos) return false;

  float l = 0.0f, r = 0.0f;
  if (std::sscanf(line.c_str() + start,
        "ODOMETRY: L_vel=%f rad/s R_vel=%f rad/s", &l, &r) != 2)
  {
    return false;
  }

  // Reject obvious garbage (NaN/inf from a corrupt line)
  if (!std::isfinite(l) || !std::isfinite(r)) return false;

  left_vel  = static_cast<double>(l);
  right_vel = static_cast<double>(r);
  return true;
}

bool CommonHardwareInterface::request_duplo_count(double & count)
{
  try {
    serial_.FlushInputBuffer();
    serial_.Write("DUPLO_COUNT\n");
    std::string line;

    serial_.ReadLine(line, '\n', 50);

    return parse_duplo_count(line, count);

  } catch (const LibSerial::ReadTimeout &) {
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial read error: %s", e.what());
    return false;
  }
}

bool CommonHardwareInterface::parse_duplo_count(
  const std::string & line, double & count)
{
  std::size_t start = line.find("DUPLO_COUNT:");
  if (start == std::string::npos) return false;

  int c = 0;
  if (std::sscanf(line.c_str() + start, "DUPLO_COUNT: %d", &c) != 1) {
    return false;
  }

  if (c < 0 || c > 255) return false;

  count = static_cast<double>(c);
  return true;
}

double CommonHardwareInterface::_ramp(double current, double target) const
{
  double delta = target - current;
  if (std::abs(delta) <= ramp_step_) return target;
  return current + ramp_step_ * (delta > 0.0 ? 1.0 : -1.0);
}

double CommonHardwareInterface::_clamp(double value) const
{
  return std::max(-max_rad_s_, std::min(max_rad_s_, value));
}

bool CommonHardwareInterface::decode_duplo_count_request(double v) const
{
  int is_requested = static_cast<int>(std::round(v));
  return is_requested == 1;  // Only "Collect" mode requests duplo count updates
}

}  // namespace j2cdynamics_driver