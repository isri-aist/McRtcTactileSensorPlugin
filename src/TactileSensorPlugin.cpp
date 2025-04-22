#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/DataStore.h>

#include <McRtcTactileSensorPlugin/TactileSensorPlugin.h>
#include <eigen_conversions/eigen_msg.h>
#include <functional>

namespace
{
std::vector<Eigen::Vector3d> getRectVertices(const std::vector<Eigen::Vector3d> & points)
{
  if(points.empty())
  {
    return {};
  }

  Eigen::Vector2d vertexMin = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector2d vertexMax = Eigen::Vector2d::Constant(std::numeric_limits<double>::lowest());

  for(const auto & point : points)
  {
    vertexMin = vertexMin.cwiseMin(point.head<2>());
    vertexMax = vertexMax.cwiseMax(point.head<2>());
  }

  return {Eigen::Vector3d(vertexMax.x(), vertexMax.y(), 0.0), Eigen::Vector3d(vertexMin.x(), vertexMax.y(), 0.0),
          Eigen::Vector3d(vertexMin.x(), vertexMin.y(), 0.0), Eigen::Vector3d(vertexMax.x(), vertexMin.y(), 0.0)};
}
} // namespace

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
#if ENABLE_MUJOCO
        sensorInfo.msgType = MsgType::Mujoco;
#else
        mc_rtc::log::error_and_throw(
            "[mc_plugin::TactileSensorPlugin] MuJoCo is disabled. Rebuild with the \"-DENABLE_MUJOCO=ON\" option.");
#endif
      }
      else if(msgTypeStr == "eskin")
      {
#if ENABLE_ESKIN
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

    if(sensorConfig.has("tactileSensorList"))
    {
      for(const auto & tactileSensorConfig : sensorConfig("tactileSensorList"))
      {
        TactileSensorInfo tactileSensorInfo;

        tactileSensorInfo.topicName = static_cast<std::string>(tactileSensorConfig("topicName"));
        tactileSensorInfo.tactileSensorFrameName =
            static_cast<std::string>(tactileSensorConfig("tactileSensorFrameName"));

        sensorInfo.tactileSensorInfoList.push_back(tactileSensorInfo);
      }
    }
    else
    {
      TactileSensorInfo tactileSensorInfo;

      if(!sensorConfig.has("topicName"))
      {
        mc_rtc::log::error_and_throw(
            "[mc_plugin::TactileSensorPlugin] The \"topicName\" key must be specified in the sensor configuration.");
      }
      tactileSensorInfo.topicName = static_cast<std::string>(sensorConfig("topicName"));

      if(!sensorConfig.has("tactileSensorFrameName"))
      {
        mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] The \"tactileSensorFrameName\" key must be "
                                     "specified in the sensor configuration.");
      }
      tactileSensorInfo.tactileSensorFrameName = static_cast<std::string>(sensorConfig("tactileSensorFrameName"));
      if(!robot.hasFrame(tactileSensorInfo.tactileSensorFrameName))
      {
        mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] No frame named \"{}\" exists.",
                                     tactileSensorInfo.tactileSensorFrameName);
      }

      sensorInfo.tactileSensorInfoList.push_back(tactileSensorInfo);
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

    sensorConfig("forceScale", sensorInfo.forceScale);
    sensorConfig("contactForceThre", sensorInfo.contactForceThre);
    sensorConfig("contactProxThre", sensorInfo.contactProxThre);

    sensorInfoList_.push_back(sensorInfo);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  sensorSubList_.clear();
  for(size_t sensorIdx = 0; sensorIdx < sensorInfoList_.size(); sensorIdx++)
  {
    const auto & sensorInfo = sensorInfoList_[sensorIdx];
    for(size_t tactileSensorIdx = 0; tactileSensorIdx < sensorInfo.tactileSensorInfoList.size(); tactileSensorIdx++)
    {
      const auto & tactileSensorInfo = sensorInfo.tactileSensorInfoList[tactileSensorIdx];
#if ENABLE_MUJOCO
      if(sensorInfo.msgType == MsgType::Mujoco)
      {
        sensorSubList_.push_back(nh_->subscribe<mujoco_tactile_sensor_plugin::TactileSensorData>(
            tactileSensorInfo.topicName, 1,
            std::bind(&TactileSensorPlugin::mujocoSensorCallback, this, std::placeholders::_1, sensorIdx,
                      tactileSensorIdx)));
      }
#endif
#if ENABLE_ESKIN
      if(sensorInfo.msgType == MsgType::Eskin)
      {
        sensorSubList_.push_back(
            nh_->subscribe<eskin_ros_utils::PatchData>(tactileSensorInfo.topicName, 1,
                                                       std::bind(&TactileSensorPlugin::eskinSensorCallback, this,
                                                                 std::placeholders::_1, sensorIdx, tactileSensorIdx)));
      }
#endif
    }
  }

  reset(gc);

  mc_rtc::log::success("[mc_plugin] Initialized TactileSensorPlugin.");
  for(const auto & sensorInfo : sensorInfoList_)
  {
    mc_rtc::log::success("  - Force sensor: {}", sensorInfo.forceSensorName);
    for(const auto & tactileSensorInfo : sensorInfo.tactileSensorInfoList)
    {
      mc_rtc::log::success("    - Tactile sensor: {} ({})", tactileSensorInfo.topicName,
                           tactileSensorInfo.tactileSensorFrameName);
    }
  }
}

void TactileSensorPlugin::reset(mc_control::MCGlobalController & // gc
)
{
  sensorDataList_.clear();
  for(const auto & sensorInfo : sensorInfoList_)
  {
    SensorData sensorData;
    sensorData.tactileSensorDataList.resize(sensorInfo.tactileSensorInfoList.size());
    sensorDataList_.push_back(sensorData);
  }
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
    const auto & sensorData = sensorDataList_[sensorIdx];
    auto & forceSensor = robot.data()->forceSensors.at(robot.data()->forceSensorsIndex.at(sensorInfo.forceSensorName));

    // Transform from tactile sensor frame to force sensor frame
    sva::PTransformd forceSensorPose = forceSensor.X_fsmodel_fsactual() * forceSensor.X_0_f(robot);
    sva::ForceVecd wrenchInForceSensorFrame = sva::ForceVecd::Zero();
    std::vector<Eigen::Vector3d> convexVerticesInForceSensorFrame;

    for(size_t tactileSensorIdx = 0; tactileSensorIdx < sensorInfo.tactileSensorInfoList.size(); tactileSensorIdx++)
    {
      const auto & tactileSensorInfo = sensorInfo.tactileSensorInfoList[tactileSensorIdx];
      const auto & tactileSensorData = sensorData.tactileSensorDataList[tactileSensorIdx];

      sva::PTransformd tactileSensorPose = robot.frame(tactileSensorInfo.tactileSensorFrameName).position();
      sva::PTransformd tactileToForceTrans = forceSensorPose * tactileSensorPose.inv();

      wrenchInForceSensorFrame += tactileToForceTrans.dualMul(tactileSensorData.wrench);

      for(const auto & vertex : tactileSensorData.convexVertices)
      {
        sva::PTransformd vertexPose =
            sva::PTransformd(Eigen::Matrix3d::Identity(), Eigen::Vector3d(vertex.x(), vertex.y(), 0.0));
        convexVerticesInForceSensorFrame.push_back((vertexPose * tactileToForceTrans.inv()).translation());
      }
    }

    forceSensor.wrench(wrenchInForceSensorFrame);
    setDatastore<std::vector<Eigen::Vector3d>>(controller.datastore(), sensorInfo.forceSensorName + "::convexVertices",
                                               convexVerticesInForceSensorFrame);
  }
}

template<class ValueType>
void TactileSensorPlugin::setDatastore(mc_rtc::DataStore & datastore, const std::string & key, const ValueType & value)
{
  if(datastore.has(key))
  {
    datastore.assign<ValueType>(key, value);
  }
  else
  {
    datastore.make<ValueType>(key, value);
  }
}

#if ENABLE_MUJOCO
void TactileSensorPlugin::mujocoSensorCallback(
    const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg,
    size_t sensorIdx,
    size_t tactileSensorIdx)
{
  const auto & sensorInfo = sensorInfoList_[sensorIdx];

  // Calculate wrench by integrating tactile sensor measurements
  sva::ForceVecd wrench = sva::ForceVecd::Zero();
  std::vector<Eigen::Vector3d> contactPoints;
  for(size_t i = 0; i < sensorMsg->forces.size(); i++)
  {
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    tf::pointMsgToEigen(sensorMsg->positions[i], position);
    tf::vectorMsgToEigen(sensorMsg->normals[i], normal);
    Eigen::Vector3d force = sensorInfo.forceScale * sensorMsg->forces[i] * normal;
    Eigen::Vector3d moment = position.cross(force);
    wrench.force() += force;
    wrench.moment() += moment;

    if(force.z() > sensorInfo.contactForceThre)
    {
      contactPoints.push_back(position);
    }
  }

  auto & tactileSensorData = sensorDataList_[sensorIdx].tactileSensorDataList[tactileSensorIdx];
  tactileSensorData.wrench = wrench;
  tactileSensorData.convexVertices = getRectVertices(contactPoints);
}
#endif

#if ENABLE_ESKIN
void TactileSensorPlugin::eskinSensorCallback(const eskin_ros_utils::PatchData::ConstPtr & sensorMsg,
                                              size_t sensorIdx,
                                              size_t tactileSensorIdx)
{
  const auto & sensorInfo = sensorInfoList_[sensorIdx];

  // Calculate wrench by integrating tactile sensor measurements
  sva::ForceVecd wrench = sva::ForceVecd::Zero();
  std::vector<Eigen::Vector3d> contactPoints;
  for(const auto & cellMsg : sensorMsg->cells)
  {
    Eigen::Vector3d position;
    Eigen::Quaterniond quat;
    tf::pointMsgToEigen(cellMsg.pose.position, position);
    tf::quaternionMsgToEigen(cellMsg.pose.orientation, quat);
    Eigen::Vector3d normal = -1.0 * quat.toRotationMatrix().col(2);
    // Edit here to change the conversion calibration from e-Skin tactile measurements to force
    Eigen::Vector3d force =
        sensorInfo.forceScale * (cellMsg.forces[0] + cellMsg.forces[1] + cellMsg.forces[2]) * normal;
    Eigen::Vector3d moment = position.cross(force);
    wrench.force() += force;
    wrench.moment() += moment;

    if(force.z() > sensorInfo.contactForceThre || cellMsg.prox > sensorInfo.contactProxThre)
    {
      contactPoints.push_back(position);
    }
  }

  auto & tactileSensorData = sensorDataList_[sensorIdx].tactileSensorDataList[tactileSensorIdx];
  tactileSensorData.wrench = wrench;
  tactileSensorData.convexVertices = getRectVertices(contactPoints);
}
#endif
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("TactileSensor", mc_plugin::TactileSensorPlugin)
