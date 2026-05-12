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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# libserial — system library installed via apt
find_library(LIBSERIAL_LIB serial REQUIRED)

# ── Build the plugin library ─────────────────────────────────────────────────
add_library(arduino_hardware_interface SHARED
  src/arduino_hardware_interface.cpp
)

target_include_directories(arduino_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arduino_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(arduino_hardware_interface ${LIBSERIAL_LIB})

# ── Export plugin description ────────────────────────────────────────────────
pluginlib_export_plugin_description_file(
  hardware_interface arduino_hardware_interface.xml
)

# ── Install ──────────────────────────────────────────────────────────────────
install(
  TARGETS arduino_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES arduino_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# ── Ament export ─────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(arduino_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()