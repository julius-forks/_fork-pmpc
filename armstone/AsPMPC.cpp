#include <geometry_msgs/TransformStamped.h>

#include <armstone/AsPMPC.h>

#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <perceptive_mpc/costs/VoxbloxCost.h>

#include <perceptive_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>

using namespace perceptive_mpc;

AsPMPC::AsPMPC(const ros::NodeHandle &nh)
    : nh_(nh), mpcUpdateFailed_(false), planAvailable_(false), kinematicInterfaceConfig_() {}

//TODO:
// Check old joint state and stop if old.
// interactive  goal pub  full on sim., arm only  in real.
// check  joy stick left trigger. (axes 5) if not pressed stop. deadman switch - done

// Mutex cheat sheet
// boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_); //write mutex
// boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_); //Read mutex

bool AsPMPC::run()
{
  ROS_INFO("Loading Params");
  parseParameters();
  loadTransforms();

  // ROS API
  ROS_INFO("Initialise ROS API");
  goalPoseSubscriber_ = nh_.subscribe("desired_end_effector_pose_topic", 1, &AsPMPC::desiredEndEffectorPoseCb, this); // For motion of arm only.
  taskTrajectorySubscriber_ = nh_.subscribe("trajectory_cmd_topic", 1, &AsPMPC::taskTrajectoryCmdCb, this);
  joySubscriber_ = nh_.subscribe("/ui/joy", 1, &AsPMPC::joyCb, this);
  armJointVelPub_ = nh_.advertise<std_msgs::Float64MultiArray>("armjointvelcmd_topic", 1);
  baseTwistPub_ = nh_.advertise<geometry_msgs::Twist>("basetwistcmd_topic", 1);
  armStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
  pointsOnRobotPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("collision_points", 1, false);
  endEffectorPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("est_ee_pose", 100);

  // Initialise MPCInterface
  ROS_INFO("Initialise MPCInterface");
  AsPerceptiveMpcInterfaceConfig config;
  config.taskFileName = mpcTaskFile_;
  config.kinematicsInterface = std::make_shared<asArmKinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
  config.voxbloxConfig = configureCollisionAvoidance(config.kinematicsInterface);
  pmpcInterface_.reset(new AsPerceptiveMpcInterface(config));
  mpcInterface_ = std::make_shared<MpcInterface>(pmpcInterface_->getMpc());
  mpcInterface_->reset();  

  //Setting initial state
  if (sim_mode_)
  {
    observation_.state() = pmpcInterface_->getInitialState();
    observation_.time() = ros::Time::now().toSec();
  }
  else
  {
    firstObservationUpdated_ = false;
    jointStatesSubscriber_ =
        nh_.subscribe("joint_states", 1, &AsPMPC::jointStatesCb, this);
    ROS_INFO("Waiting for obeservation to be updated");
    ros::Rate _r(10); // 10 hz
    while (ros::ok())
    {
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(lastDeadManTimeMutex_); //Read mutex
        if (jointStatesSubscriber_.getNumPublishers() >0 && firstObservationUpdated_ && lastDeadManTime_>1.0)
        {
          break;
        }
      }      
      ros::spinOnce();
      _r.sleep();
    }
  }
  //observation_ should now be updated by the joint cb or set via initial  
  ROS_INFO("Observation was updated; Setting up MPC loops");
  optimalState_ = observation_.state();
  setCurrentObservation(observation_);

  ROS_INFO_STREAM("Starting from initial state: " << observation_.state().transpose());
  ROS_INFO_STREAM("Initial time (delta): " << observation_.time());

  initializeCostDesiredTrajectory();

  // Tracker worker
  std::thread trackerWorker(&AsPMPC::trackerLoop, this, ros::Rate(controlLoopFrequency_));

  // Mpc update worker
  std::thread mpcUpdateWorker(&AsPMPC::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));

  if (sim_mode_)
  {    
    tfUpdateWorker_ = new std::thread(&AsPMPC::tfUpdate, this, ros::Rate(tfUpdateFrequency_));
  }
  
  //Thread monitor
  std::thread loopMonitorWorker(&AsPMPC::loopMonitor, this, ros::Rate(1.));
  ros::spin();//Blocks 
  trackerWorker.join();//Waits for threads
  mpcUpdateWorker.join();
  if (sim_mode_)
  {
    tfUpdateWorker_->join();
  }
  loopMonitorWorker.join();
  return true;
}


void AsPMPC::parseParameters()
{
  ros::NodeHandle pNh("~");
  // Kinematics:
  kinematicInterfaceConfig_.baseMass = pNh.param<double>("base_mass", 35);
  auto _v = pNh.param<std::vector<double>>("base_com", {0, 0, 0});
  kinematicInterfaceConfig_.baseCOM = Eigen::Vector3d(_v[0], _v[1], _v[2]);
  // Modes:
  sim_mode_ = pNh.param<bool>("kinematic_simulation_mode", true);
  collision_mode_ = pNh.param<bool>("collision_avoidance_mode", false);
  // Frames
  end_effector_frame_ = pNh.param<std::string>("end_effector_frame", "ee");
  base_frame_ = pNh.param<std::string>("base_frame", "base_link_footprint");
  odom_frame_ = pNh.param<std::string>("odom_frame", "odom");
  // Files
  mpcTaskFile_ = pNh.param<std::string>("mpc_task_file", "task.info");
  // MPC params
  mpcUpdateFrequency_ = pNh.param<double>("mpc_update_frequency", 100);
  tfUpdateFrequency_ = pNh.param<double>("tf_update_frequency", 10);
  maxLinearVelocity_ = pNh.param<double>("max_linear_velocity", 1.0);
  maxAngularVelocity_ = pNh.param<double>("max_angular_velocity", 1.0);
  controlLoopFrequency_ = pNh.param<double>("control_loop_frequency", 100);
  // Other params
  deadmanAxes_ = pNh.param<double>("deadman_axes", 4);
  lastDeadManTime_=0.;
}

bool AsPMPC::trackerLoop(ros::Rate rate)
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

      // MAKE A COPY OF OBSERVATION
      Observation observation;
      {

        if (sim_mode_)
        {
          boost::unique_lock<boost::shared_mutex> lockGuard(observationMutex_); //write mutex
          observation_.state() = optimalState_;
          observation_.time() = ros::Time::now().toSec();
          observation = observation_;
        }
        else
        {
          boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_); //Read mutex
          observation = observation_;                                           // sets current observation to last one received
        }
      }

      MpcInterface::input_vector_t controlInput;
      MpcInterface::state_vector_t optimalState;
      size_t subsystem;
      try
      {
        if (sim_mode_)
        {
          mpcInterface_->updatePolicy();
          mpcInterface_->evaluatePolicy(observation.time(), observation.state(), optimalState, controlInput, subsystem);
        }
        else
        {
          mpcInterface_->updatePolicy();
          mpcInterface_->evaluatePolicy(ros::Time::now().toSec(), observation.state(), optimalState, controlInput, subsystem);
          pubControlInput(controlInput);
        }
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
                                        << "    run time:          " << observation.time()  << std::endl
                                        << "    current_state_base_q: " << observation.state().transpose().head<4>() << std::endl
                                        << "    current_state_base_xyz: " << observation.state().transpose().head<7>().tail<3>() << std::endl
                                        << "    current_state_arm_joints: " << observation.state().transpose().tail<6>() << std::endl
                                        << "    optimalState_base_q:  " << optimalState.transpose().head<4>() << std::endl
                                        << "    optimalState_base_xyz:  " << optimalState.transpose().head<7>().tail<3>() << std::endl
                                        << "    optimalState_arm_joints:  " << optimalState.transpose().tail<6>() << std::endl
                                        << "    controlInput_base_xyth:  " << controlInput.transpose().head<3>() << std::endl
                                        << "    controlInput_arm_vel:  " << controlInput.transpose().tail<6>() << std::endl
                                        << "    Full Control INput:  " << controlInput.transpose() << std::endl
                                        << std::endl);
      optimalState_ = optimalState;
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsPMPC::trackerLoop]. Message: " << ex.what());
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsPMPC::trackerLoop]. Message: " << ex.what());
      return false;
    }   
    {
      boost::unique_lock<boost::shared_mutex> lockGuard1(trackerLoopCountMutex_);
      trackerLoopCount_++;
    }
    
    rate.sleep();
  }
  return true;
}

bool AsPMPC::mpcUpdate(ros::Rate rate)
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
      ROS_ERROR_STREAM("Caught exception while calling [AsPMPC::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsPMPC::mpcUpdate]. Message: " << ex.what());
      mpcUpdateFailed_ = true;
      return false;
    }
    planAvailable_ = true;
    {
      boost::unique_lock<boost::shared_mutex> lockGuard1(mpcLoopCountMutex_);
      mpcLoopCount_++;
    }
    rate.sleep();
  }
  return true;
}

bool AsPMPC::tfUpdate(ros::Rate rate)
{
  while (ros::ok())
  {
    try
    {
      Observation currentObservation;
      {
        boost::shared_lock<boost::shared_mutex> lockGuard(observationMutex_);
        currentObservation = observation_;
      }
      publishBaseTransform(currentObservation);
      publishArmState(currentObservation);
      publishEndEffectorPose();
      if (pointsOnRobot_)
      {
        pointsOnRobotPublisher_.publish(pointsOnRobot_->getVisualization(currentObservation.state()));
      }

    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR_STREAM("runtime_error occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsKinematicSimulation::tfUpdate]. Message: " << ex.what());
      return false;
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("exception occured!");
      ROS_ERROR_STREAM("Caught exception while calling [AsKinematicSimulation::tfUpdate]. Message: " << ex.what());
      return false;
    }

    {
      boost::unique_lock<boost::shared_mutex> lockGuard1(tfLoopCountMutex_);
      tfLoopCount_++;
    }
    rate.sleep();
  }
  return true;
}

void AsPMPC::setCurrentObservation(const Observation &observation)
{
  //Copies observation into mpc
  // the quaternion is not closed under addition
  // the state integration will make the quaternion non unique as the simulatoin goes on
  Eigen::Quaterniond currentBaseRotation = Eigen::Quaterniond(observation.state().head<4>());
  currentBaseRotation.normalize();
  mpcInterface_->setCurrentObservation(observation);
}

void AsPMPC::initializeCostDesiredTrajectory()
{
  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  reference_vector_t reference = reference_vector_t::Zero();
  auto currentEndEffectorPose = getEndEffectorPose();
  auto currentBasePose = getBasePose();
  // Eigen::Quaterniond qe = Eigen::Quaterniond(currentEndEffectorPose.getRotation().getUnique().toImplementation().coeffs());
  // Eigen::Quaterniond qb = Eigen::Quaterniond(currentBasePose.getRotation().getUnique().toImplementation().coeffs());
  // qe.normalize();
  // qb.normalize();

  reference.head<Definitions::POSE_DIM>().head<4>() = currentEndEffectorPose.getRotation().getUnique().toImplementation().coeffs() ;
  reference.head<Definitions::POSE_DIM>().tail<3>() = currentEndEffectorPose.getPosition().toImplementation();
  reference.tail<Definitions::BASE_STATE_DIM_>().head<4>() = currentBasePose.getRotation().getUnique().toImplementation().coeffs() ;
  reference.tail<Definitions::BASE_STATE_DIM_>().tail<3>() = currentBasePose.getPosition().toImplementation();

  Observation observation;
  {
    boost::shared_lock<boost::shared_mutex> observationLock(observationMutex_);
    observation = observation_;
  }

  costDesiredTrajectories_.desiredTimeTrajectory().push_back(observation.time());
  costDesiredTrajectories_.desiredTimeTrajectory().push_back(observation.time() + 10);
  costDesiredTrajectories_.desiredStateTrajectory().push_back(reference);
  costDesiredTrajectories_.desiredStateTrajectory().push_back(reference);
  costDesiredTrajectories_.desiredInputTrajectory().push_back(InputVector::Zero());
  costDesiredTrajectories_.desiredInputTrajectory().push_back(InputVector::Zero());
  costDesiredTrajectories_.display();
}


bool AsPMPC::isDead()
{

  {
    boost::shared_lock<boost::shared_mutex> lockGuard(lastDeadManTimeMutex_); //Read mutex
    if (ros::Time::now().toSec() > lastDeadManTime_ + 0.15)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "DEADMAN SWITCH RELEASED. Stopping input publishing");
      return true;
    }
  }

  if(!sim_mode_){
    boost::shared_lock<boost::shared_mutex> lockGuard(lastJointStateTimeMutex_); //Read mutex
    if (ros::Time::now().toSec() > lastJointStateTime_ + 0.15)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Joint states msg old. Stopping input publishing");
      return true;
    }
  }

  return false;
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
        ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
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
        ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
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