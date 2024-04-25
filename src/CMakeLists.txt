set(AUTOLOAD_TactileSensor_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load TactileSensor plugin")
add_plugin(TactileSensor TactileSensorPlugin.cpp)
target_link_libraries(
  TactileSensor PUBLIC PkgConfig::eigen_conversions
                       PkgConfig::mujoco_tactile_sensor_plugin)
if(ENABLE_ESKIN)
  target_compile_definitions(TactileSensor PUBLIC ENABLE_ESKIN)
  target_include_directories(TactileSensor
                             PUBLIC ${eskin_ros_utils_INCLUDE_DIRS})
  target_link_directories(TactileSensor PUBLIC ${eskin_ros_utils_LIBRARY_DIRS})
  target_link_libraries(TactileSensor PUBLIC ${eskin_ros_utils_LIBRARIES})
endif()
