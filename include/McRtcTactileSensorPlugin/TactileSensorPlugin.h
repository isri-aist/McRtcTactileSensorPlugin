#pragma once

#include <mc_control/GlobalPlugin.h>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>

#if ENABLE_ESKIN
#  include <eskin_ros_utils/msg/patch_data.hpp>
#endif
#if ENABLE_MUJOCO
#  include <mujoco_tactile_sensor_plugin/msg/tactile_sensor_data.hpp>
#endif

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

  /** \brief Tactile sensor information. */
  struct TactileSensorInfo
  {
    //! Topic name
    std::string topicName;

    //! Frame name of tactile sensor
    std::string tactileSensorFrameName;
  };

  /** \brief Sensor information. */
  struct SensorInfo
  {
    //! Message type
    MsgType msgType = MsgType::Mujoco;

    std::vector<TactileSensorInfo> tactileSensorInfoList;

    //! Force sensor name
    std::string forceSensorName;

    //! Force scale (this value is multiplied to the tactile measurement value)
    double forceScale = 1.0;

    //! Force threshold for determining contact in each cell [N]
    double contactForceThre = 3.0;

    //! Proximity threshold for determining contact in each cell
    double contactProxThre = 1.0;
  };

  /** \brief Tactile sensor data. */
  struct TactileSensorData
  {
    //! Wrench (represented in the frame of tactile sensor)
    sva::ForceVecd wrench = sva::ForceVecd::Zero();

    //! Vertices of the convex hull of the contact region (represented in the frame of tactile sensor)
    std::vector<Eigen::Vector3d> convexVertices;
  };

  /** \brief Sensor data. */
  struct SensorData
  {
    std::vector<TactileSensorData> tactileSensorDataList;
  };

public:
  /** \brief Destructor. */
  ~TactileSensorPlugin();

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
  /** \brief Set datastore entry. */
  template<class ValueType>
  void setDatastore(mc_rtc::DataStore & datastore, const std::string & key, const ValueType & value);

#if ENABLE_MUJOCO
  /** \brief ROS callback of tactile sensor topic from MuJoCo.
      \param sensorMsg sensor message
      \param sensorIdx sensor index
      \param tactileSensorIdx tactile sensor index
   */
  void mujocoSensorCallback(const mujoco_tactile_sensor_plugin::msg::TactileSensorData::ConstSharedPtr & sensorMsg,
                            size_t sensorIdx,
                            size_t tactileSensorIdx);
#endif

#if ENABLE_ESKIN
  /** \brief ROS callback of tactile sensor topic from e-Skin.
      \param sensorMsg sensor message
      \param sensorIdx sensor index
      \param tactileSensorIdx tactile sensor index
     */
  void eskinSensorCallback(const eskin_ros_utils::msg::PatchData::ConstSharedPtr & sensorMsg,
                           size_t sensorIdx,
                           size_t tactileSensorIdx);
#endif

protected:
  //! Sensor information list
  std::vector<SensorInfo> sensorInfoList_;

  //! Sensor data list
  std::vector<SensorData> sensorDataList_;

  //! ROS variables
  //! @{
  bool ownsRclcppContext_ = false;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> sensorSubList_;
  //! @}
};

} // namespace mc_plugin
