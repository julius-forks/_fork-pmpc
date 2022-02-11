#include <geometry_msgs/TransformStamped.h>

#include <armstone/AsPMPC.h>

#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <perceptive_mpc/costs/VoxbloxCost.h>

#include <perceptive_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>

using namespace perceptive_mpc;

bool AsPMPC::loopMonitor(ros::Rate rate)
{
  while (ros::ok())
  {
    double time = ros::Time::now().toSec();
    double dt = time - monitorTimeLast_;

    {
      mpcLoopRate_ = (int)std::round(((double)mpcLoopCount_) / dt);
      trackerLoopRate_ = (int)std::round(((double)trackerLoopCount_) / dt);
      tfLoopLoopRate_ = (int)std::round(((double)tfLoopCount_) / dt);
      obsRate_ = (int)std::round(((double)obsCount_) / dt);
      obsCount_ = 0;
      tfLoopCount_ = 0;
      trackerLoopCount_ = 0;
      mpcLoopCount_ = 0;

      ROS_INFO_STREAM(std::endl
                      << "    MPC loop rate:     " << mpcLoopRate_ << std::endl
                      << "    tracker loop rate: " << trackerLoopRate_ << std::endl
                      << "    TF loop rate:     " << tfLoopLoopRate_ << std::endl
                      << "    obs  rate:     " << obsRate_ << std::endl
                      << std::endl);
      monitorTimeLast_ = time;
    }

    checkDead(); //run constant check dead.
    if (isDead_)
    {
      reinitMpc_ = true;
    }
    rate.sleep();
  }
  return true;
}

void AsPMPC::checkDead()
{

  {
    boost::shared_lock<boost::shared_mutex> lockGuard(lastDeadManTimeMutex_); //Read mutex
    if (ros::Time::now().toSec() > lastDeadManTime_ + 0.15)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "DEADMAN SWITCH RELEASED. Stopping input publishing");
      isDead_ = true; // dead
      return;
    }
  }

  if (!sim_mode_)
  {
    boost::shared_lock<boost::shared_mutex> lockGuard(lastJointStateTimeMutex_); //Read mutex
    if (ros::Time::now().toSec() > lastJointStateTime_ + 0.15)
    {
      isDead_ = true; // dead
      ROS_WARN_STREAM_THROTTLE(1.0, "Joint states msg old. Stopping input publishing");
      return;
    }
  }

  isDead_ = false; //not dead
}

kindr::HomTransformQuatD AsPMPC::getEndEffectorPose()
{
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    Eigen::Matrix<double, 4, 4> endEffectorToWorldTransform;
    Eigen::VectorXd currentState = observation_.state();

    asArmKinematics<double> kinematics(kinematicInterfaceConfig_);
    kinematics.computeState2EndeffectorTransform(endEffectorToWorldTransform, currentState);
    Eigen::Quaterniond eigenBaseRotation(endEffectorToWorldTransform.topLeftCorner<3, 3>());
    return kindr::HomTransformQuatD(kindr::Position3D(endEffectorToWorldTransform.topRightCorner<3, 1>()),
                                    kindr::RotationQuaternionD(eigenBaseRotation));
  }
}

kindr::HomTransformQuatD AsPMPC::getBasePose()
{
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
    const Eigen::Quaterniond currentRotation = Eigen::Quaterniond(observation_.state().head<Definitions::BASE_STATE_DIM_>().head<4>());
    const Eigen::Matrix<double, 3, 1> currentPosition = observation_.state().head<Definitions::BASE_STATE_DIM_>().tail<3>();

    return kindr::HomTransformQuatD(kindr::Position3D(currentPosition),
                                    kindr::RotationQuaternionD(currentRotation));
  }
}

std::shared_ptr<VoxbloxCostConfig> AsPMPC::configureCollisionAvoidance(
    std::shared_ptr<KinematicsInterfaceAD> kinematicInterface)
{
  ros::NodeHandle pNh("~");
  std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig = nullptr;

  if (pNh.hasParam("collision_points"))
  {
    perceptive_mpc::PointsOnRobot::points_radii_t pointsAndRadii(8);
    using pair_t = std::pair<double, double>;

    XmlRpc::XmlRpcValue collisionPoints;
    pNh.getParam("collision_points", collisionPoints);
    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("collision_points parameter is not of type array.");
      return voxbloxCostConfig;
    }
    for (int i = 0; i < collisionPoints.size(); i++)
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN_STREAM("collision_points[" << i << "] parameter is not of type array.");
        return voxbloxCostConfig;
      }
      for (int j = 0; j < collisionPoints[i].size(); j++)
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] parameter is not of type array.");
          return voxbloxCostConfig;
        }
        if (collisionPoints[i][j].size() != 2)
        {
          ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] does not have 2 elements.");
          return voxbloxCostConfig;
        }
        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
    perceptive_mpc::PointsOnRobotConfig config;
    config.pointsAndRadii = pointsAndRadii;
    using ad_type = CppAD::AD<CppAD::cg::CG<double>>;
    config.kinematics = kinematicInterface;
    pointsOnRobot_.reset(new perceptive_mpc::PointsOnRobot(config));

    if (pointsOnRobot_->numOfPoints() > 0)
    {
      voxbloxCostConfig.reset(new VoxbloxCostConfig());
      voxbloxCostConfig->pointsOnRobot = pointsOnRobot_;

      esdfCachingServer_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));
      voxbloxCostConfig->interpolator = esdfCachingServer_->getInterpolator();

      pointsOnRobot_->initialize("points_on_robot");
    }
    else
    {
      // if there are no points defined for collision checking, set this pointer to null to disable the visualization
      pointsOnRobot_ = nullptr;
    }
  }
  return voxbloxCostConfig;
}

void AsPMPC::loadTransforms()
{
  tfListener_ = new tf2_ros::TransformListener(tfBuffer_, true);
  asArmKinematics<double> kinematics(kinematicInterfaceConfig_);

  ROS_INFO("Waiting for expected tf frames");

  ros::Rate _r(10); // 10 hz
  while (ros::ok())
  {
    ros::spinOnce();
    _r.sleep();

    {
      geometry_msgs::TransformStamped transformStamped;
      try
      {
        transformStamped = tfBuffer_.lookupTransform(base_frame_, kinematics.armMountLinkName(), ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_ERROR_THROTTLE(1.0, "pmpc: %s", ex.what());
        continue;
      }
      Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                              transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

      kinematicInterfaceConfig_.transformBase_X_ArmMount = Eigen::Matrix4d::Identity();
      kinematicInterfaceConfig_.transformBase_X_ArmMount.block<3, 3>(0, 0) = quat.toRotationMatrix();

      kinematicInterfaceConfig_.transformBase_X_ArmMount(0, 3) = transformStamped.transform.translation.x;
      kinematicInterfaceConfig_.transformBase_X_ArmMount(1, 3) = transformStamped.transform.translation.y;
      kinematicInterfaceConfig_.transformBase_X_ArmMount(2, 3) = transformStamped.transform.translation.z;
      ROS_INFO_STREAM("baseToArmMount_: " << std::endl
                                          << kinematicInterfaceConfig_.transformBase_X_ArmMount);
    }

    {
      geometry_msgs::TransformStamped transformStamped;
      try
      {
        transformStamped = tfBuffer_.lookupTransform(kinematics.toolMountLinkName(), end_effector_frame_, ros::Time(0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_ERROR_THROTTLE(1.0, "pmpc:  %s", ex.what());
        continue;
      }
      Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                              transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

      kinematicInterfaceConfig_.transformToolMount_X_Endeffector = Eigen::Matrix4d::Identity();
      kinematicInterfaceConfig_.transformToolMount_X_Endeffector.block<3, 3>(0, 0) = quat.toRotationMatrix();

      kinematicInterfaceConfig_.transformToolMount_X_Endeffector(0, 3) = transformStamped.transform.translation.x;
      kinematicInterfaceConfig_.transformToolMount_X_Endeffector(1, 3) = transformStamped.transform.translation.y;
      kinematicInterfaceConfig_.transformToolMount_X_Endeffector(2, 3) = transformStamped.transform.translation.z;
      ROS_INFO_STREAM("wrist2ToEETransform_: " << std::endl
                                               << kinematicInterfaceConfig_.transformToolMount_X_Endeffector);
    }
    ROS_INFO("Frames found");
    break;
  }
}