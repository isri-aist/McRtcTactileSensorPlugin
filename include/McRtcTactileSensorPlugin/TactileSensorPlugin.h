#pragma once

#include <mc_control/GlobalPlugin.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#if ENABLE_ESKIN
#  include <eskin_ros_utils/PatchData.h>
#endif
#if ENABLE_MUJOCO
#  include <mujoco_tactile_sensor_plugin/TactileSensorData.h>
#endif
#include <variant>

namespace mc_plugin
{

/** \brief Tactile sensor plugin. */
struct TactileSensorPlugin : public mc_control::GlobalPlugin
{
protected:
  /** \brief ROS message type for tactile sensor. */
  enum class MsgType
  {
    //! Message from the tactile sensor plugin in MuJoCo
    Mujoco = 0,

    //! Message from the e-Skin tactile sensor via eSkinRosUtils
    Eskin
  };

  /** \brief Sensor information. */
  struct SensorInfo
  {
    //! Message type
    MsgType msgType = MsgType::Mujoco;

    //! Topic name
    std::string topicName;

    //! Frame name of tactile sensor
    std::string tactileSensorFrameName;

    //! Force sensor name
    std::string forceSensorName;
  };

public:
  /** \brief Returns the plugin configuration. */
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  /** \brief Initialize the plugin. */
  void init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config) override;

  /** \brief Reset the plugin. */
  void reset(mc_control::MCGlobalController & gc) override;

  /** \brief Run before mc_control::MCGlobalController::run() */
  void before(mc_control::MCGlobalController & gc) override;

  /** \brief Run after mc_control::MCGlobalController::run() */
  inline void after(mc_control::MCGlobalController & gc) override{};

protected:
#if ENABLE_MUJOCO
  /** \brief ROS callback of tactile sensor topic from MuJoCo.
      \param sensorMsg sensor message
      \param sensorIdx sensor index
   */
  void mujocoSensorCallback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg,
                            size_t sensorIdx);
#endif

#if ENABLE_ESKIN
  /** \brief ROS callback of tactile sensor topic from e-Skin.
        \param sensorMsg sensor message
        \param sensorIdx sensor index
     */
  void eskinSensorCallback(const eskin_ros_utils::PatchData::ConstPtr & sensorMsg, size_t sensorIdx);
#endif

protected:
  //! Sensor information list
  std::vector<SensorInfo> sensorInfoList_;

  //! Sensor message list
#if ENABLE_MUJOCO && ENABLE_ESKIN
  std::vector<std::variant<std::nullptr_t,
                           std::shared_ptr<mujoco_tactile_sensor_plugin::TactileSensorData>,
                           std::shared_ptr<eskin_ros_utils::PatchData>>>
#elif ENABLE_MUJOCO && !ENABLE_ESKIN
  std::vector<std::variant<std::nullptr_t, std::shared_ptr<mujoco_tactile_sensor_plugin::TactileSensorData>>>
#elif !ENABLE_MUJOCO && ENABLE_ESKIN
  std::vector<std::variant<std::nullptr_t, std::shared_ptr<eskin_ros_utils::PatchData>>>
#else
  std::vector<std::variant<std::nullptr_t>>
#endif
      sensorMsgList_;

  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  std::vector<ros::Subscriber> sensorSubList_;
  //! @}
};

} // namespace mc_plugin
