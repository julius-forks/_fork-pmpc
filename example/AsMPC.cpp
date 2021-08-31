/*
 * Copyright (c) 2020 Johannes Pankert <pankertj@ethz.ch>, Giuseppe Rizzi
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of this work nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <geometry_msgs/TransformStamped.h>

#include <example/AsMPC.h>

#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <perceptive_mpc/costs/VoxbloxCost.h>

#include <perceptive_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>

using namespace perceptive_mpc;

AsMPC::AsMPC(const ros::NodeHandle &nh)
    : nh_(nh), mpcUpdateFailed_(false), planAvailable_(false), kinematicInterfaceConfig_() {}

bool AsMPC::run()
{
  parseParameters();
  loadTransforms();

  PerceptiveMpcInterfaceConfig config;
  config.taskFileName = mpcTaskFile_;
  config.kinematicsInterface = std::make_shared<asArmKinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
  config.voxbloxConfig = configureCollisionAvoidance(config.kinematicsInterface);
  pmpcInterface_.reset(new PerceptiveMpcInterface(config));
  mpcInterface_ = std::make_shared<MpcInterface>(pmpcInterface_->getMpc());
  mpcInterface_->reset();
  observation_.state() = pmpcInterface_->getInitialState();
  observation_.time() = ros::Time::now().toSec();
  optimalState_ = observation_.state();
  initialTime_ = observation_.time();

  setCurrentObservation(observation_);
  ROS_INFO_STREAM("Starting from initial state: " << observation_.state().transpose());
  ROS_INFO_STREAM("Initial time (delta): " << observation_.time() - initialTime_);

  initializeCostDesiredTrajectory();

  // Init ros stuff
  jointStatesSubscriber_ =
      nh_.subscribe("joint_states", 1, &AsMPC::jointStatesCb, this);
  ROS_INFO("Waiting for joint states to begin ...");
  while (ros::ok() && jointStatesSubscriber_.getNumPublishers() == 0)
  {
    ros::Rate(100).sleep();
  }
  ROS_INFO("Joint states are flowing");

  goalPoseSubscriber_ =
      nh_.subscribe("/perceptive_mpc/desired_end_effector_pose", 1, &AsMPC::desiredEndEffectorPoseCb, this);

  // Tracker worker
  std::thread trackerWorker(&AsMPC::trackerLoop, this, ros::Rate(controlLoopFrequency_));

  // Mpc update worker
  mpcUpdateFrequency_ = (mpcUpdateFrequency_ == -1) ? 100 : mpcUpdateFrequency_;
  std::thread mpcUpdateWorker(&AsMPC::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));

  ros::spin();
  trackerWorker.join();
  mpcUpdateWorker.join();
  return true;
}

void AsMPC::loadTransforms()
{
  tf2_ros::Buffer tfBuffer_();
  tf2_ros::TransformListener tfListener_(tfBuffer_);

  asArmKinematics<double> kinematics(kinematicInterfaceConfig_);
  
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer_.lookupTransform(base_frame_, kinematics.armMountLinkName(), ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      throw;
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
      transformStamped = tfBuffer_.lookupTransform(kinematics.toolMountLinkName(), end_effector_frame_, ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      throw;
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
}

void AsMPC::parseParameters()
{
  ros::NodeHandle pNh("~");

  kinematicInterfaceConfig_.baseMass = pNh.param<double>("base_mass", 35);

  auto _v = pNh.param<std::vector<double>>("base_com", {0, 0, 0});
  kinematicInterfaceConfig_.baseCOM = Eigen::Vector3d(_v[0], _v[1], _v[2]);
  end_effector_frame_ = pNh.param<std::string>("end_effector_frame", "ee");
  base_frame_ = pNh.param<std::string>("base_frame", "base_link");

  mpcTaskFile_ = pNh.param<std::string>("mpc_task_file", "task.info");
  // Seems these are in EE frame
  mpcUpdateFrequency_ = pNh.param<double>("mpc_update_frequency", 100);
  rosPublishFrequency_ = pNh.param<double>("tf_update_frequency", 10);
  maxLinearVelocity_ = pNh.param<double>("max_linear_velocity", 1.0);
  maxAngularVelocity_ = pNh.param<double>("max_angular_velocity", 1.0);
  controlLoopFrequency_ = pNh.param<double>("control_loop_frequency", 100);
  auto defaultForceStd = pNh.param<std::vector<double>>("default_external_force", std::vector<double>());
  if (defaultForceStd.size() == 3)
  {
    defaultForce_ = Eigen::Vector3d::Map(defaultForceStd.data(), 3);
  }
  auto defaultTorqueStd = pNh.param<std::vector<double>>("default_external_torque", std::vector<double>());
  if (defaultTorqueStd.size() == 3)
  {
    defaultTorque_ = Eigen::Vector3d::Map(defaultTorqueStd.data(), 3);
  }
}

bool AsMPC::trackerLoop(ros::Rate rate)
{
  while (ros::ok())
  {
    try
    {
      if (mpcUpdateFailed_)
      {
        ROS_ERROR_STREAM("Mpc update failed, stop.");
        return false;
      }

      if (!planAvailable_)
      {
        rate.sleep();
        continue;
      }

      // use the optimal state as the next observation initialized with first observation
      // TODO: for integration on hardware, write the current observation from the state estimator instead
      Observation observation;
      {
        boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_);
        observation = observation_;
      }

      MpcInterface::input_vector_t controlInput;
      MpcInterface::state_vector_t optimalState;
      size_t subsystem;
      try
      {
        mpcInterface_->updatePolicy();
        mpcInterface_->evaluatePolicy(observation.time(), observation.state(), optimalState, controlInput, subsystem);
        // TODO: for integration on hardware, send the computed control inputs to the motor controllers
      }
      catch (const std::runtime_error &ex)
      {
        ROS_ERROR_STREAM("runtime_error occured!");
        ROS_ERROR_STREAM("Caught exception while calling [mpcInterface_->evaluatePolicy]. Message: " << ex.what());
        return false;
      }
      catch (const std::exception &ex)
      {
        ROS_ERROR_STREAM("exception occured!");
        ROS_ERROR_STREAM("Caught exception while calling [mpcInterface_->evaluatePolicy]. Message: " << ex.what());
        return false;
      }

      ROS_INFO_STREAM_THROTTLE(5.0, std::endl
                                        << "    time:          " << observation.time() - initialTime_ << std::endl
                                        << "    current_state: " << observation.state().transpose() << std::endl
                                        << "    optimalState:  " << optimalState.transpose() << std::endl
                                        << "    controlInput:  " << controlInput.transpose() << std::endl
                                        << std::endl);
      optimalState_ = optimalState;
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPC::trackerLoop]. Message: " << ex.what());
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPC::trackerLoop]. Message: " << ex.what());
      return false;
    }
    rate.sleep();
  }
  return true;
}

bool AsMPC::mpcUpdate(ros::Rate rate)
{
  while (ros::ok())
  {
    if (mpcUpdateFailed_)
    {
      rate.sleep();
      continue;
    }

    try
    {
      {
        // TODO: uncomment for admittance control on hardware:
        //        auto adaptedCostDesiredTrajectory = costDesiredTrajectories_;
        //        kindr::WrenchD measuredWrench; // input measured wrench from sensor here
        //        admittanceReferenceModule.adaptPath(rate.cycleTime().toSec(), adaptedCostDesiredTrajectory.desiredStateTrajectory(),
        //        measuredWrench); mpcInterface_->setTargetTrajectories(adaptedCostDesiredTrajectory);
        boost::shared_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
        mpcInterface_->setTargetTrajectories(costDesiredTrajectories_);
      }
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        setCurrentObservation(observation_);
      }
      if (esdfCachingServer_)
      {
        esdfCachingServer_->updateInterpolator();
      }
      mpcInterface_->advanceMpc();
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPC::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPC::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    planAvailable_ = true;
    rate.sleep();
  }
  return true;
}

void AsMPC::setCurrentObservation(const Observation &observation)
{
  // the quaternion is not closed under addition
  // the state integration will make the quaternion non unique as the simulatoin goes on
  Eigen::Quaterniond currentBaseRotation = Eigen::Quaterniond(observation.state().head<4>());
  currentBaseRotation.normalize();
  mpcInterface_->setCurrentObservation(observation);
}

void AsMPC::initializeCostDesiredTrajectory()
{
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  reference_vector_t reference = reference_vector_t::Zero();
  auto currentEndEffectorPose = getEndEffectorPose();
  reference.head<Definitions::POSE_DIM>().head<4>() = currentEndEffectorPose.getRotation().getUnique().toImplementation().coeffs();
  reference.head<Definitions::POSE_DIM>().tail<3>() = currentEndEffectorPose.getPosition().toImplementation();
  reference.tail<Definitions::WRENCH_DIM>().head<3>() = defaultForce_;
  reference.tail<Definitions::WRENCH_DIM>().tail<3>() = defaultTorque_;

  Observation observation;
  {
    boost::shared_lock<boost::shared_mutex> observationLock(observationMutex_);
    observation = observation_;
  }

  costDesiredTrajectories_.desiredTimeTrajectory().push_back(observation.time());
  costDesiredTrajectories_.desiredTimeTrajectory().push_back(observation.time() + 1);
  costDesiredTrajectories_.desiredStateTrajectory().push_back(reference);
  costDesiredTrajectories_.desiredStateTrajectory().push_back(reference);
  costDesiredTrajectories_.desiredInputTrajectory().push_back(InputVector::Zero());
  costDesiredTrajectories_.desiredInputTrajectory().push_back(InputVector::Zero());
}

void AsMPC::desiredEndEffectorPoseCb(const geometry_msgs::PoseStampedConstPtr &msgPtr)
{
  geometry_msgs::Pose currentEndEffectorPose;
  kindr_ros::convertToRosGeometryMsg(getEndEffectorPose(), currentEndEffectorPose);

  perceptive_mpc::WrenchPoseTrajectory wrenchPoseTrajectory;
  wrenchPoseTrajectory.header.stamp = ros::Time::now();
  wrenchPoseTrajectory.posesWrenches.resize(2);
  wrenchPoseTrajectory.posesWrenches[0].header.stamp = wrenchPoseTrajectory.header.stamp;
  wrenchPoseTrajectory.posesWrenches[0].pose = currentEndEffectorPose;
  tf2::toMsg(defaultForce_, wrenchPoseTrajectory.posesWrenches[0].wrench.force);
  tf2::toMsg(defaultTorque_, wrenchPoseTrajectory.posesWrenches[0].wrench.torque);

  wrenchPoseTrajectory.posesWrenches[1].header.stamp = wrenchPoseTrajectory.header.stamp;
  wrenchPoseTrajectory.posesWrenches[1].pose = msgPtr->pose;
  tf2::toMsg(defaultForce_, wrenchPoseTrajectory.posesWrenches[1].wrench.force);
  tf2::toMsg(defaultTorque_, wrenchPoseTrajectory.posesWrenches[1].wrench.torque);

  desiredWrenchPoseTrajectoryCb(wrenchPoseTrajectory);
}

void AsMPC::jointStatesCb(const sensor_msgs::JointState &msgPtr)
{
  geometry_msgs::TransformStamped ts;
    try
    {
      ts = tfBuffer_.lookupTransform(odom_frame_, base_frame_, ros::Time.now(), ros::Duration(0.15));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      throw;
    }

  {      
    boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
    observation_.state(0) = ts.transform.rotation.x;
    observation_.state(1) = ts.transform.rotation.y;
    observation_.state(2) = ts.transform.rotation.z;
    observation_.state(3) = ts.transform.rotation.w;
    observation_.state(4) = ts.transform.translation.x;
    observation_.state(5) = ts.transform.translation.y;
    observation_.state(6) = ts.transform.translation.z;
    observation_.state(7) = msgPtr.position.at(0);
    observation_.state(8) = msgPtr.position.at(1);
    observation_.state(9) = msgPtr.position.at(2);
    observation_.state(10) = msgPtr.position.at(3);
    observation_.state(11) = msgPtr.position.at(4);
    observation_.state(12) = msgPtr.position.at(5);
    observation_.time() = msgPtr.header.stamp;
  }
}

void AsMPC::pubControlInput(const sensor_msgs::JointState &msgPtr)
{
//  base twist
//  arm joint states
}

kindr::HomTransformQuatD AsMPC::getEndEffectorPose()
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

std::shared_ptr<VoxbloxCostConfig> AsMPC::configureCollisionAvoidance(
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