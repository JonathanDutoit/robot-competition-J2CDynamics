cmake_minimum_required(VERSION 3.16)
project(j2cdynamics_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_library(LIBSERIAL_LIB serial REQUIRED)

# ─────────────────────────────────────────────
# 1. HARDWARE PLUGIN
# ─────────────────────────────────────────────
add_library(common_hardware_interface SHARED
  src/common_hardware_interface.cpp
  src/da_hardware_interface.cpp
  src/do_hardware_interface.cpp
)
target_include_directories(common_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(common_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

target_link_libraries(common_hardware_interface ${LIBSERIAL_LIB})

pluginlib_export_plugin_description_file(
  hardware_interface
  common_hardware_interface.xml
)

install(TARGETS common_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES common_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# ─────────────────────────────────────────────
# 2. JOY MODE MAPPER NODE
# ─────────────────────────────────────────────
add_executable(joy_mode_mapper
  src/joy_mode_mapper.cpp)

ament_target_dependencies(joy_mode_mapper
  rclcpp
  sensor_msgs
  std_msgs)

install(TARGETS joy_mode_mapper
  DESTINATION lib/${PROJECT_NAME})

# ── Ament export ─────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(common_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# ─────────────────────────────────────────────
ament_package()