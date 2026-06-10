#pragma once

#include "j2cdynamics_driver/common_hardware_interface.hpp"

namespace j2cdynamics_driver
{

class DoHardwareInterface : public CommonHardwareInterface
{
public:
  DoHardwareInterface() = default;
  ~DoHardwareInterface() override = default;
};

}  // namespace j2cdynamics_driver