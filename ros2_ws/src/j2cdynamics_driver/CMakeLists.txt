find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# libserial — installed via apt, not a ROS package
find_library(LIBSERIAL_LIB serial REQUIRED)

add_library(arduino_hardware_interface SHARED
  src/arduino_hardware_interface.cpp
)
target_include_directories(arduino_hardware_interface PUBLIC
  include
)
ament_target_dependencies(arduino_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(arduino_hardware_interface ${LIBSERIAL_LIB})

pluginlib_export_plugin_description_file(
  hardware_interface arduino_hardware_interface.xml
)

install(TARGETS arduino_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)