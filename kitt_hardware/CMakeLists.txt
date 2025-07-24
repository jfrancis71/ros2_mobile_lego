cmake_minimum_required(VERSION 3.16)
project(kitt_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

if(NOT DEFINED ENV{BRICKPI3_ROOT_DIR})
	message(FATAL_ERROR "You must set BRICKPI3_ROOT_DIR")
endif()

include_directories("$ENV{BRICKPI3_ROOT_DIR}/Software/C")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  kitt_hardware
  SHARED
  src/kitt_hardware.cpp
)
target_compile_features(kitt_hardware PUBLIC cxx_std_17)
target_include_directories(kitt_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/kitt_hardware>
)
ament_target_dependencies(
  kitt_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kitt_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/kitt_hardware
)
install(TARGETS kitt_hardware
  EXPORT export_kitt_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_kitt_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
