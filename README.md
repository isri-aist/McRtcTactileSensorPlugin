# [McRtcTactileSensorPlugin](https://github.com/isri-aist/McRtcTactileSensorPlugin)
mc_rtc plugin to use tactile sensor in controller via ROS interface

[![CI](https://github.com/isri-aist/McRtcTactileSensorPlugin/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/McRtcTactileSensorPlugin/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/McRtcTactileSensorPlugin/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/McRtcTactileSensorPlugin)](https://github.com/isri-aist/McRtcTactileSensorPlugin/blob/master/LICENSE)

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic`

### Dependencies
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)
- [isri-aist/MujocoTactileSensorPlugin](https://github.com/isri-aist/MujocoTactileSensorPlugin) (Install as a ROS package)
- [eSkinRosUtils](https://github.com/isri-aist/eSkinRosUtils)
   - An option to build without dependency on the e-Skin tactile sensor is also supported.

### Installation procedure
```bash
# Load the ROS setup file so that the `mujoco_tactile_sensor_plugin` package can be found
$ mkdir ${HOME}/src && cd ${HOME}/src
$ git clone git@github.com:isri-aist/McRtcTactileSensorPlugin.git --recursive
$ cd McRtcTactileSensorPlugin
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo # Add "-DENABLE_ESKIN=OFF" option if you do not use e-Skin tactile sensor
$ make
$ make install
```

## Plugins
### TactileSensor
This plugin receives tactile sensor data via ROS topic and sets wrench on the robot's force sensor in mc_rtc controller.

See [this page](https://jrl-umi3218.github.io/mc_rtc/tutorials/usage/global-plugins.html) for a general tutorial on configuring mc_rtc plugins.

Put the following line in the mc_rtc configuration (e.g. `${HOME}/.config/mc_rtc/mc_rtc.yaml`).
```yaml
Plugins: [TactileSensor]
```

Put the following lines in the plugin configuration (e.g. `${HOME}/.config/mc_rtc/plugins/TactileSensor.yaml`).
```yaml
sensors:
  - msgType: mujoco # "mujoco" or "eskin"
    topicName: /mujoco/tactile_sensor
    tactileSensorFrameName: jvrc1_right_elbow_tactile_sensor
    forceSensorName: RightElbowForceSensor
```

The above configuration assumes a ROS topic named `/mujoco/tactile_sensor` with a message of type [mujoco_tactile_sensor_plugin/TactileSensorData](https://github.com/isri-aist/MujocoTactileSensorPlugin/blob/main/msg/TactileSensorData.msg) being published by some node (e.g. [isri-aist/MujocoTactileSensorPlugin](https://github.com/isri-aist/MujocoTactileSensorPlugin)).
The tactile sensor frame (e.g. `jvrc1_right_elbow_tactile_sensor`) must be defined in a URDF model (e.g. `jvrc_description/urdf/jvrc1.urdf`).
The force sensor (e.g. `RightElbowForceSensor`) must be defined in robot module (e.g. `mc_rtc/src/mc_robots/jvrc1.cpp`).

For system details for using tactile sensor with mc_mujoco, please refer to [here](doc/McMujocoTactileSensorSytem.md).
