#include <mc_control/GlobalPluginMacros.h>

#include <McRtcTactileSensorPlugin/TactileSensorPlugin.h>
#include <eigen_conversions/eigen_msg.h>
#include <functional>

namespace mc_plugin
{

mc_control::GlobalPlugin::GlobalPluginConfiguration TactileSensorPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void TactileSensorPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  const auto & controller = gc.controller();
  const auto & robot = controller.robot();

  // Load configuration
  if(!config.has("sensors"))
  {
    mc_rtc::log::error_and_throw(
        "[mc_plugin::TactileSensorPlugin] The \"sensors\" key must be specified in the configuration.");
  }
  for(const auto & sensorConfig : config("sensors"))
  {
    SensorInfo sensorInfo;

    if(sensorConfig.has("msgType"))
    {
      std::string msgTypeStr = sensorConfig("msgType");
      if(msgTypeStr == "mujoco")
      {
        sensorInfo.msgType = MsgType::Mujoco;
      }
      else if(msgTypeStr == "eskin")
      {
#ifdef ENABLE_ESKIN
        sensorInfo.msgType = MsgType::Eskin;
#else
        mc_rtc::log::error_and_throw(
            "[mc_plugin::TactileSensorPlugin] e-Skin is disabled. Rebuild with the \"-DENABLE_ESKIN=ON\" option.");
#endif
      }
      else
      {
        mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] Unsupported message type: " + msgTypeStr);
      }
    }
    if(!sensorConfig.has("topicName"))
    {
      mc_rtc::log::error_and_throw(
          "[mc_plugin::TactileSensorPlugin] The \"topicName\" key must be specified in the sensor configuration.");
    }
    sensorInfo.topicName = static_cast<std::string>(sensorConfig("topicName"));
    if(!sensorConfig.has("tactileSensorFrameName"))
    {
      mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] The \"tactileSensorFrameName\" key must be "
                                   "specified in the sensor configuration.");
    }
    sensorInfo.tactileSensorFrameName = static_cast<std::string>(sensorConfig("tactileSensorFrameName"));
    if(!robot.hasFrame(sensorInfo.tactileSensorFrameName))
    {
      mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] No frame named \"{}\" exists.",
                                   sensorInfo.tactileSensorFrameName);
    }
    if(!sensorConfig.has("forceSensorName"))
    {
      mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] The \"forceSensorName\" key must be specified in "
                                   "the sensor configuration.");
    }
    sensorInfo.forceSensorName = static_cast<std::string>(sensorConfig("forceSensorName"));
    if(!robot.hasForceSensor(sensorInfo.forceSensorName))
    {
      mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] No force sensor named \"{}\" exists.",
                                   sensorInfo.forceSensorName);
    }

    sensorInfoList_.push_back(sensorInfo);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  sensorSubList_.clear();
  for(size_t sensorIdx = 0; sensorIdx < sensorInfoList_.size(); sensorIdx++)
  {
    if(sensorInfoList_[sensorIdx].msgType == MsgType::Mujoco)
    {
      sensorSubList_.push_back(nh_->subscribe<mujoco_tactile_sensor_plugin::TactileSensorData>(
          sensorInfoList_[sensorIdx].topicName, 1,
          std::bind(&TactileSensorPlugin::mujocoSensorCallback, this, std::placeholders::_1, sensorIdx)));
    }
#ifdef ENABLE_ESKIN
    else // if(sensorInfoList_[sensorIdx].msgType == MsgType::Eskin)
    {
      sensorSubList_.push_back(nh_->subscribe<eskin_ros_utils::PatchData>(
          sensorInfoList_[sensorIdx].topicName, 1,
          std::bind(&TactileSensorPlugin::eskinSensorCallback, this, std::placeholders::_1, sensorIdx)));
    }
#endif
  }

  reset(gc);

  mc_rtc::log::success("[mc_plugin] Initialized TactileSensorPlugin.");
}

void TactileSensorPlugin::reset(mc_control::MCGlobalController & // gc
)
{
  sensorMsgList_.clear();
  sensorMsgList_.resize(sensorSubList_.size(), nullptr);
}

void TactileSensorPlugin::before(mc_control::MCGlobalController & gc)
{
  auto & controller = gc.controller();
  auto & robot = controller.robot();

  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  // Set measured wrench
  for(size_t sensorIdx = 0; sensorIdx < sensorInfoList_.size(); sensorIdx++)
  {
    const auto & sensorInfo = sensorInfoList_[sensorIdx];
    auto & forceSensor = robot.data()->forceSensors.at(robot.data()->forceSensorsIndex.at(sensorInfo.forceSensorName));

    sva::ForceVecd wrench = sva::ForceVecd::Zero();
    if(std::holds_alternative<std::nullptr_t>(sensorMsgList_[sensorIdx]))
    {
      // Do nothing
    }
    else if(std::holds_alternative<std::shared_ptr<mujoco_tactile_sensor_plugin::TactileSensorData>>(
                sensorMsgList_[sensorIdx]))
    {
      const auto & sensorMsg =
          std::get<std::shared_ptr<mujoco_tactile_sensor_plugin::TactileSensorData>>(sensorMsgList_[sensorIdx]);

      // Calculate wrench by integrating tactile sensor data
      for(size_t i = 0; i < sensorMsg->forces.size(); i++)
      {
        Eigen::Vector3d position;
        Eigen::Vector3d normal;
        tf::pointMsgToEigen(sensorMsg->positions[i], position);
        tf::vectorMsgToEigen(sensorMsg->normals[i], normal);
        Eigen::Vector3d force = sensorMsg->forces[i] * normal;
        Eigen::Vector3d moment = position.cross(force);
        wrench.force() += force;
        wrench.moment() += moment;
      }

      // Transform from tactile sensor frame to force sensor frame
      sva::PTransformd tactileSensorPose = robot.frame(sensorInfo.tactileSensorFrameName).position();
      sva::PTransformd forceSensorPose = forceSensor.X_fsmodel_fsactual() * forceSensor.X_0_f(robot);
      sva::PTransformd tactileToForceTrans = forceSensorPose * tactileSensorPose.inv();
      wrench = tactileToForceTrans.dualMul(wrench);
    }
#ifdef ENABLE_ESKIN
    else if(std::holds_alternative<std::shared_ptr<eskin_ros_utils::PatchData>>(sensorMsgList_[sensorIdx]))
    {
      const auto & sensorMsg = std::get<std::shared_ptr<eskin_ros_utils::PatchData>>(sensorMsgList_[sensorIdx]);

      // Calculate wrench by integrating tactile sensor data
      for(const auto & cellMsg : sensorMsg->cells)
      {
        Eigen::Vector3d position;
        Eigen::Quaterniond quat;
        tf::pointMsgToEigen(cellMsg.pose.position, position);
        tf::quaternionMsgToEigen(cellMsg.pose.orientation, quat);
        Eigen::Vector3d normal = -1.0 * quat.toRotationMatrix().col(2);
        // \todo Calibrate force measurements from e-Skin
        Eigen::Vector3d force = 10.0 * (cellMsg.forces[0] + cellMsg.forces[1] + cellMsg.forces[2]) * normal;
        Eigen::Vector3d moment = position.cross(force);
        wrench.force() += force;
        wrench.moment() += moment;
      }

      // Transform from tactile sensor frame to force sensor frame
      sva::PTransformd tactileSensorPose = robot.frame(sensorInfo.tactileSensorFrameName).position();
      sva::PTransformd forceSensorPose = forceSensor.X_fsmodel_fsactual() * forceSensor.X_0_f(robot);
      sva::PTransformd tactileToForceTrans = forceSensorPose * tactileSensorPose.inv();
      wrench = tactileToForceTrans.dualMul(wrench);
    }
#endif
    forceSensor.wrench(wrench);
  }
}

void TactileSensorPlugin::mujocoSensorCallback(
    const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg,
    size_t sensorIdx)
{
  sensorMsgList_[sensorIdx] = std::make_shared<mujoco_tactile_sensor_plugin::TactileSensorData>(*sensorMsg);
}

#ifdef ENABLE_ESKIN
void TactileSensorPlugin::eskinSensorCallback(const eskin_ros_utils::PatchData::ConstPtr & sensorMsg, size_t sensorIdx)
{
  sensorMsgList_[sensorIdx] = std::make_shared<eskin_ros_utils::PatchData>(*sensorMsg);
}
#endif
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("TactileSensor", mc_plugin::TactileSensorPlugin)
