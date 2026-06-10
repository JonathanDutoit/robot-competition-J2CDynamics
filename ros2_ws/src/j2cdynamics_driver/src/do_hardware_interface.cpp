#include "j2cdynamics_driver/do_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"

// ── Plugin registration ───────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  j2cdynamics_driver::DoHardwareInterface,
  hardware_interface::SystemInterface)