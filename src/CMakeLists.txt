set(AUTOLOAD_TactileSensor_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load TactileSensor plugin")
add_plugin(TactileSensor TactileSensorPlugin.cpp)
target_link_libraries(TactileSensor PUBLIC rclcpp::rclcpp tf2_eigen::tf2_eigen)

if(ENABLE_MUJOCO)
  target_compile_definitions(TactileSensor PUBLIC ENABLE_MUJOCO=1)
  target_link_libraries(TactileSensor
                        PUBLIC
                               mujoco_tactile_sensor_plugin::mujoco_tactile_sensor_plugin__rosidl_typesupport_cpp)
else()
  target_compile_definitions(TactileSensor PUBLIC ENABLE_MUJOCO=0)
endif()

if(ENABLE_ESKIN)
  target_compile_definitions(TactileSensor PUBLIC ENABLE_ESKIN=1)
  target_link_libraries(
    TactileSensor
    PUBLIC eskin_ros_utils::eskin_ros_utils__rosidl_typesupport_cpp)
else()
  target_compile_definitions(TactileSensor PUBLIC ENABLE_ESKIN=0)
endif()
