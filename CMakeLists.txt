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
option(ENABLE_MUJOCO "Enable MuJoCo tactile sensor" ON)
option(ENABLE_ESKIN "Enable e-Skin tactile sensor" ON)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

find_package(rclcpp REQUIRED CONFIG)
find_package(tf2_eigen REQUIRED CONFIG)

function(check_rosidl_typesupport_cpp_target package_name)
  if(NOT TARGET
     ${package_name}::${package_name}__rosidl_typesupport_cpp)
    message(
      FATAL_ERROR
        "The ROS 2 package ${package_name} was found, but the expected C++ typesupport target "
        "${package_name}::${package_name}__rosidl_typesupport_cpp was not exported."
    )
  endif()
endfunction()

if(ENABLE_MUJOCO)
  # mujoco_tactile_sensor_plugin
  find_package(mujoco_tactile_sensor_plugin REQUIRED CONFIG)
  check_rosidl_typesupport_cpp_target(mujoco_tactile_sensor_plugin)
  message("-- Enable MuJoCo")
else()
  message("-- Disable MuJoCo")
endif()

if(ENABLE_ESKIN)
  # eskin_ros_utils
  find_package(eskin_ros_utils REQUIRED CONFIG)
  check_rosidl_typesupport_cpp_target(eskin_ros_utils)
  message("-- Enable e-Skin")
else()
  message("-- Disable e-Skin")
endif()

install(FILES etc/TactileSensor.yaml
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

add_subdirectory(src)

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
