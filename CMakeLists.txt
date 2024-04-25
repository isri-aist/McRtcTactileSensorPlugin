cmake_minimum_required(VERSION 3.13)

set(PROJECT_NAME McRtcTactileSensorPlugin)
set(PROJECT_VERSION 0.1.0)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/McRtcTactileSensorPlugin)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_EXPORT_NO_TARGET TRUE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

# eigen_conversions
pkg_check_modules(eigen_conversions REQUIRED IMPORTED_TARGET eigen_conversions)

# mujoco_tactile_sensor_plugin
find_package(PkgConfig)
pkg_check_modules(mujoco_tactile_sensor_plugin REQUIRED IMPORTED_TARGET
                  mujoco_tactile_sensor_plugin)
pkg_check_modules(eskin_ros_utils REQUIRED IMPORTED_TARGET eskin_ros_utils)

install(FILES etc/TactileSensor.yaml
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

add_subdirectory(src)

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
