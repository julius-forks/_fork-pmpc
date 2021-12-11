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

#include <armstone/AsMPCController.h>

#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include <perceptive_mpc/costs/VoxbloxCost.h>

#include <perceptive_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>

using namespace perceptive_mpc;

AsMPCController::AsMPCController(const ros::NodeHandle &nh)
    : nh_(nh), mpcUpdateFailed_(false), planAvailable_(false), kinematicInterfaceConfig_() {}

bool AsMPCController::run()
{

  // Init ros stuff
  initialTime_ = ros::Time::now().toSec();
  tfListener_ = new tf2_ros::TransformListener(tfBuffer_, true);
  jointStatesSubscriber_ =
      nh_.subscribe("joint_states", 1, &AsMPCController::jointStatesCb, this);
  ROS_INFO("Waiting for joint states to begin ...");
  while (ros::ok() && jointStatesSubscriber_.getNumPublishers() == 0)
  {
    ros::Rate(100).sleep();
  }
  ROS_INFO("Joint states are flowing");

  //Assure starting state is initial state. i.e. we have a recent observation
  ROS_INFO("Waiting for obeservation to be updated");
  ros::Rate r(10); // 10 hz
  firstObservationUpdated_ = false;
  while (ros::ok() && !firstObservationUpdated_)
  {
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Observation was updated; initialising PMPC");

  goalPoseSubscriber_ = nh_.subscribe("/perceptive_mpc/desired_end_effector_pose", 1, &AsMPCController::desiredEndEffectorPoseCb, this);
  taskTrajectorySubscriber_ = nh_.subscribe("/print_tasker/trajectory_cmd", 1, &AsMPCController::taskTrajectoryCmdCb, this);
  armJointVelPub_ = nh_.advertise<std_msgs::Float64MultiArray>("armjointvelcmd_topic", 1);
  baseTwistPub_ = nh_.advertise<geometry_msgs::Twist>("basetwistcmd_topic", 1);

  parseParameters();
  loadTransforms();

  // PMPC stuff
  AsPerceptiveMpcInterfaceConfig config;
  config.taskFileName = mpcTaskFile_;
  config.kinematicsInterface = std::make_shared<asArmKinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
  config.voxbloxConfig = configureCollisionAvoidance(config.kinematicsInterface);
  pmpcInterface_.reset(new AsPerceptiveMpcInterface(config));
  mpcInterface_ = std::make_shared<MpcInterface>(pmpcInterface_->getMpc());
  mpcInterface_->reset();
  // observation_.state() = pmpcInterface_->getInitialState();
  // observation_.time() = ros::Time::now().toSec();
  //Update optimal and imitial states. NOTE these will be overwritten by ROS
  ROS_INFO("Updating optimal and inital state to current state.");
  optimalState_ = observation_.state();
  setCurrentObservation(observation_);

  ROS_INFO_STREAM("Starting from initial state: " << observation_.state().transpose());
  ROS_INFO_STREAM("Initial time (delta): " << observation_.time() - initialTime_);

  initializeCostDesiredTrajectory();

  // Tracker worker
  std::thread trackerWorker(&AsMPCController::trackerLoop, this, ros::Rate(controlLoopFrequency_));

  // Mpc update worker
  mpcUpdateFrequency_ = (mpcUpdateFrequency_ == -1) ? 100 : mpcUpdateFrequency_;
  std::thread mpcUpdateWorker(&AsMPCController::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));

  ros::spin();
  trackerWorker.join();
  mpcUpdateWorker.join();
  return true;
}

void AsMPCController::loadTransforms()
{

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

void AsMPCController::parseParameters()
{
  ros::NodeHandle pNh("~");

  kinematicInterfaceConfig_.baseMass = pNh.param<double>("base_mass", 35);

  auto _v = pNh.param<std::vector<double>>("base_com", {0, 0, 0});
  kinematicInterfaceConfig_.baseCOM = Eigen::Vector3d(_v[0], _v[1], _v[2]);
  end_effector_frame_ = pNh.param<std::string>("end_effector_frame", "ee");
  base_frame_ = pNh.param<std::string>("base_frame", "base_link");
  odom_frame_ = pNh.param<std::string>("odom_frame", "odom");

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

  // armJointVelMsg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // armJointVelMsg_.layout.dim[0].size = 1;
  // armJointVelMsg_.layout.dim[0].stride = 1;
  // armJointVelMsg_.layout.dim[0].stride = 1;
}

bool AsMPCController::trackerLoop(ros::Rate rate)
{
  while (ros::ok())
  {
    // Will allow for disabling enabling later on when this becomes an action server or something.
    // if (mpcEnabled_)
    // {

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

      // MAKE A COPY OF OBSERVATION
      Observation observation;
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        observation = observation_;
      }

      MpcInterface::input_vector_t controlInput;
      MpcInterface::state_vector_t optimalState;
      size_t subsystem;
      try
      {
        mpcInterface_->updatePolicy();
        mpcInterface_->evaluatePolicy(ros::Time::now().toSec(), observation.state(), optimalState, controlInput, subsystem);
        // mpcInterface_->evaluatePolicy(observation.time(), observation.state(), optimalState, controlInput, subsystem);
        pubControlInput(controlInput);
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
                                        << "    Observation time:          " << observation.time() << std::endl
                                        << "    Initial Time:          " << initialTime_ << std::endl
                                        << "    run time:          " << observation.time() - initialTime_ << std::endl
                                        << "    current_state: " << observation.state().transpose() << std::endl
                                        << "    optimalState:  " << optimalState.transpose() << std::endl
                                        << "    controlInput:  " << controlInput.transpose() << std::endl
                                        << std::endl);
      optimalState_ = optimalState;
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPCController::trackerLoop]. Message: " << ex.what());
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPCController::trackerLoop]. Message: " << ex.what());
      return false;
    }
    // }

    rate.sleep();
  }
  return true;
}

bool AsMPCController::mpcUpdate(ros::Rate rate)
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
        //        kincontrolLoopFrequency_dr::WrenchD measuredWrench; // input measured wrench from sensor here
        //        admittanceReferenceModule.adaptPath(rate.cycleTime().toSec(), adaptedCostDesiredTrajectory.desiredStateTrajectory(),
        //        measuredWrench); mpcInterface_->setTargetTrajectories(adaptedCostDesiredTrajectory);
        boost::shared_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
        mpcInterface_->setTargetTrajectories(costDesiredTrajectories_);
        ROS_INFO_STREAM_THROTTLE(1.0, std::endl
                                          << "    Controlling for:          " << std::endl
                                          << "    x y z:          "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][4] << "; "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][5] << "; "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][6] << std::endl
                                          << "    qx qy qz qw:          "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][0] << "; "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][1] << "; "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][2] << "; "
                                          << costDesiredTrajectories_.desiredStateTrajectory()[0][3] << std::endl);
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
      ROS_ERROR_STREAM("Caught exception while calling [AsMPCController::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsMPCController::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    planAvailable_ = true;
    rate.sleep();
  }
  return true;
}

void AsMPCController::setCurrentObservation(const Observation &observation)
{
  // the quaternion is not closed under addition
  // the state integration will make the quaternion non unique as the simulatoin goes on
  Eigen::Quaterniond currentBaseRotation = Eigen::Quaterniond(observation.state().head<4>());
  currentBaseRotation.normalize();
  mpcInterface_->setCurrentObservation(observation);
}

void AsMPCController::initializeCostDesiredTrajectory()
{
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  reference_vector_t reference = reference_vector_t::Zero();
  auto currentEndEffectorPose = getEndEffectorPose();
  auto currentBasePose = getBasePose();
  reference.head<Definitions::POSE_DIM>().head<4>() = currentEndEffectorPose.getRotation().getUnique().toImplementation().coeffs();
  reference.head<Definitions::POSE_DIM>().tail<3>() = currentEndEffectorPose.getPosition().toImplementation();
  reference.tail<Definitions::BASE_STATE_DIM_>().head<4>() = currentBasePose.getRotation().getUnique().toImplementation().coeffs();
  reference.tail<Definitions::BASE_STATE_DIM_>().tail<3>() = currentBasePose.getPosition().toImplementation();

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

void AsMPCController::taskTrajectoryCmdCb(const m3dp_msgs::TaskTrajectory &taskTrajectory)
{
  // USECASE put 0 base pose cost if you are using this
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  int N = taskTrajectory.points.size(); // point count
  costDesiredTrajectories_.desiredStateTrajectory().resize(N);
  costDesiredTrajectories_.desiredTimeTrajectory().resize(N);
  costDesiredTrajectories_.desiredInputTrajectory().resize(N);
  
  for (int i = 0; i < N; i++)
  {
    kindr::HomTransformQuatD desiredEEPose;
    kindr::HomTransformQuatD desiredBPose;
    reference_vector_t reference;
    kindr_ros::convertFromRosGeometryMsg(taskTrajectory.points[i].ee_pose, desiredEEPose); // to kindr pose
    kindr_ros::convertFromRosGeometryMsg(taskTrajectory.points[i].base_pose, desiredBPose); // to kindr pose
    reference.head<Definitions::POSE_DIM>().head<4>() = desiredEEPose.getRotation().toImplementation().coeffs();
    reference.head<Definitions::POSE_DIM>().tail<3>() = desiredEEPose.getPosition().toImplementation();
    reference.tail<Definitions::BASE_STATE_DIM_>().head<4>() = desiredBPose.getRotation().toImplementation().coeffs();
    reference.tail<Definitions::BASE_STATE_DIM_>().tail<3>() = desiredBPose.getPosition().toImplementation();

    costDesiredTrajectories_.desiredStateTrajectory()[i] = reference; //shove into desire STATE trajecotry
    costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();

    // Deal with feasibility. This should be a check rather than retime.
    if (i == 0)
    {
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = ros::Time::now().toSec();
    }
    else
    {
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = costDesiredTrajectories_.desiredTimeTrajectory()[0] + taskTrajectory.points[i].time_from_start.toSec();
    }
    
  }
}

void AsMPCController::desiredEndEffectorPoseCb(const geometry_msgs::PoseStampedConstPtr &msgPtr)
{
  // USECASE put 0 base pose cost if you are using this
  geometry_msgs::Pose currentEndEffectorPose;
  kindr_ros::convertToRosGeometryMsg(getEndEffectorPose(), currentEndEffectorPose);  
  kindr::HomTransformQuatD desiredPose;
  kindr::HomTransformQuatD currentPose;
  kindr_ros::convertFromRosGeometryMsg(msgPtr->pose, desiredPose);
  kindr_ros::convertFromRosGeometryMsg(currentEndEffectorPose, currentPose);  

  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  int N = 2;
  costDesiredTrajectories_.desiredStateTrajectory().resize(N);
  costDesiredTrajectories_.desiredTimeTrajectory().resize(N);
  costDesiredTrajectories_.desiredInputTrajectory().resize(N);

  reference_vector_t reference0;
  reference0.head<Definitions::POSE_DIM>().head<4>() = currentPose.getRotation().toImplementation().coeffs();
  reference0.head<Definitions::POSE_DIM>().tail<3>() = currentPose.getPosition().toImplementation();

  reference_vector_t reference1;
  reference1.head<Definitions::POSE_DIM>().head<4>() = desiredPose.getRotation().toImplementation().coeffs();
  reference1.head<Definitions::POSE_DIM>().tail<3>() = desiredPose.getPosition().toImplementation();

  costDesiredTrajectories_.desiredStateTrajectory()[0] = reference0;
  costDesiredTrajectories_.desiredStateTrajectory()[1] = reference1;
  costDesiredTrajectories_.desiredInputTrajectory()[0] = MpcInterface::input_vector_t::Zero();
  costDesiredTrajectories_.desiredInputTrajectory()[1] = MpcInterface::input_vector_t::Zero();  

  auto minTimeLinear = (desiredPose.getPosition() - currentPose.getPosition()).norm() / maxLinearVelocity_;
  auto minTimeAngular = std::abs(desiredPose.getRotation().getDisparityAngle(currentPose.getRotation())) / maxAngularVelocity_;

  double segmentDuration = std::max(minTimeLinear, minTimeAngular) + 1;  

  costDesiredTrajectories_.desiredTimeTrajectory()[0] = ros::Time::now().toSec();
  costDesiredTrajectories_.desiredTimeTrajectory()[1] = costDesiredTrajectories_.desiredTimeTrajectory()[0] + segmentDuration;
  
}

void AsMPCController::jointStatesCb(const sensor_msgs::JointStateConstPtr &msgPtr)
{
  geometry_msgs::TransformStamped ts;
  try
  {
    ts = tfBuffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(0.15));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Error finding odom-->baselink frame transform");
    return;
    // ROS_ERROR("%s", ex.what());
    // throw;
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(observationMutex_);
    observation_.state(0) = ts.transform.rotation.x;
    observation_.state(1) = ts.transform.rotation.y;
    observation_.state(2) = ts.transform.rotation.z;
    observation_.state(3) = ts.transform.rotation.w;
    observation_.state(4) = ts.transform.translation.x;
    observation_.state(5) = ts.transform.translation.y;
    observation_.state(6) = ts.transform.translation.z;
    observation_.state(7) = msgPtr->position[4];
    observation_.state(8) = msgPtr->position[5];
    observation_.state(9) = msgPtr->position[6];
    observation_.state(10) = msgPtr->position[7];
    observation_.state(11) = msgPtr->position[8];
    observation_.state(12) = msgPtr->position[9];
    // observation_.time() = msgPtr->header.stamp.toSec() -initialTime_;
    // observation_.time() = msgPtr->header.stamp.toSec();
    observation_.time() = ros::Time::now().toSec();
    ;
  }

  firstObservationUpdated_ = true;
}

void AsMPCController::pubControlInput(const MpcInterface::input_vector_t &controlInput)
{

  // MpcInterface::input_vector_t controlInput;
  // {
  //   boost::shared_lock<boost::shared_mutex> lock(controlInputMutex_);
  //   controlInput = controlInput_;
  // }
  baseTwistMsg_.linear.x = controlInput[0];
  baseTwistMsg_.linear.y = controlInput[1];
  baseTwistMsg_.angular.z = controlInput[2];

  armJointVelMsg_.data.clear();
  for (int i = 3; i < 9; i++)
  {
    armJointVelMsg_.data.push_back(controlInput[i]);
  }

  baseTwistPub_.publish(baseTwistMsg_);
  armJointVelPub_.publish(armJointVelMsg_);
}

kindr::HomTransformQuatD AsMPCController::getEndEffectorPose()
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

kindr::HomTransformQuatD AsMPCController::getBasePose()
{
  {
    boost::shared_lock<boost::shared_mutex> lock(observationMutex_);    
    const Eigen::Quaterniond currentRotation = Eigen::Quaterniond(observation_.state().head<Definitions::BASE_STATE_DIM_>().head<4>());
    const Eigen::Matrix<double, 3, 1> currentPosition = observation_.state().head<Definitions::BASE_STATE_DIM_>().tail<3>();

    return kindr::HomTransformQuatD(kindr::Position3D(currentPosition),
                                    kindr::RotationQuaternionD(currentRotation));
  }
}

std::shared_ptr<VoxbloxCostConfig> AsMPCController::configureCollisionAvoidance(
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