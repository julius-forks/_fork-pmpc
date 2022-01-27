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
    : nh_(nh),
      mpcUpdateFailed_(false),
      planAvailable_(false),
      kinematicInterfaceConfig_(),
      taskTrajectoryActionServer_(nh, "PrintTrajectory", boost::bind(&AsPMPC::printTrajectoryActionCb, this, _1), false),
      isDead_(true),      
      mpcLoopRate_(0),
      trackerLoopRate_(0),
      obsRate_(0),
      mpcLoopCount_(0),
      obsCount_(0),
      lastDeadManTime_(0.),
      lastJointStateTime_(0.),
      reinitMpc_(true),
      firstObservationUpdated_(false),
      asyncSpinner_(2)
{
}

bool AsPMPC::run()
{

  ROS_INFO("Loading Params");
  parseParameters();
  loadTransforms();

  // ROS API
  ROS_INFO("Initialise ROS API");
  asyncSpinner_.start();
  goalPoseSubscriber_ = nh_.subscribe("desired_end_effector_pose_topic", 1, &AsPMPC::desiredEndEffectorPoseCb, this); // For motion of arm only.
  joySubscriber_ = nh_.subscribe("/ui/joy", 1, &AsPMPC::joyCb, this);
  pointsOnRobotPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("collision_points", 1, false);
  endEffectorPosePublisher_ = nh_.advertise<geometry_msgs::PoseStamped>("est_ee_pose", 100);
  baseTwistPub_ = nh_.advertise<geometry_msgs::Twist>("basetwistcmd_topic", 1);
  armJointVelPub_ = nh_.advertise<std_msgs::Float64MultiArray>("armjointvelcmd_topic", 1);

  // Initialise MPCInterface
  ROS_INFO("Initialise MPCInterface");
  AsPerceptiveMpcInterfaceConfig config;
  config.taskFileName = mpcTaskFile_;
  config.kinematicsInterface = std::make_shared<asArmKinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
  config.voxbloxConfig = configureCollisionAvoidance(config.kinematicsInterface);
  pmpcInterface_.reset(new AsPerceptiveMpcInterface(config));
  mpcInterface_ = std::make_shared<MpcInterface>(pmpcInterface_->getMpc());
  mpcInterface_->reset();

  std::thread loopMonitorWorker(&AsPMPC::loopMonitor, this, ros::Rate(2.));
  taskTrajectoryActionServer_.start();
  if (sim_mode_)
  {
    runsim();
  }
  else
  {
    runreal();
  }  
  loopMonitorWorker.join();
  ros::waitForShutdown();
  return true;
}

void AsPMPC::runsim()
{

  armStatePublisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  ROS_INFO("Waiting for joint rest of system to come online...");
  while (ros::ok() && armStatePublisher_.getNumSubscribers() == 0)
  {
    ros::Rate(1).sleep();
  }

  observation_.state() = pmpcInterface_->getInitialState();
  observation_.time() = ros::Time::now().toSec();
  optimalState_ = observation_.state();
  setCurrentObservation(observation_);
  initializeCostDesiredTrajectory();

  // Tracker worker
  std::thread trackerWorker(&AsPMPC::trackerLoop, this, ros::Rate(controlLoopFrequency_));
  std::thread mpcUpdateWorker(&AsPMPC::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));
  std::thread tfUpdateWorker(&AsPMPC::tfUpdate, this, ros::Rate(tfUpdateFrequency_));

  trackerWorker.join(); //Waits for threads
  mpcUpdateWorker.join();
  tfUpdateWorker.join();
  return;
}

void AsPMPC::runreal()
{
  jointStatesSubscriber_ = nh_.subscribe("joint_states", 1, &AsPMPC::jointStatesCb, this);

  ROS_INFO("Waiting for obeservation to be updated");
  while (ros::ok() && !firstObservationUpdated_)
  {
    ros::Rate(1).sleep();
  }

  setCurrentObservation(observation_);
  initializeCostDesiredTrajectory();

  std::thread trackerWorker(&AsPMPC::trackerLoop, this, ros::Rate(controlLoopFrequency_));
  std::thread mpcUpdateWorker(&AsPMPC::mpcUpdate, this, ros::Rate(mpcUpdateFrequency_));

  trackerWorker.join(); //Waits for threads
  mpcUpdateWorker.join();
  return;
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

      mpcInterface_->updatePolicy();
      mpcInterface_->evaluatePolicy(observation.time(), observation.state(), optimalState, controlInput, subsystem);
      pubControlInput(controlInput);

      ROS_INFO_STREAM_THROTTLE(5.0, std::endl
                                        << "    Observation time:          " << observation.time() << std::endl
                                        << "    run time:          " << observation.time() << std::endl
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
    trackerLoopCount_++;
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
      ROS_WARN("MPC update failed");
      rate.sleep();
      continue;
    }

    try
    {
      if (reinitMpc_)
      {
        initializeCostDesiredTrajectory();
      }
      if (trajectoryUpdated_)
      {
        boost::shared_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
        mpcInterface_->setTargetTrajectories(costDesiredTrajectories_);
        trajectoryUpdated_ = false;
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
    mpcLoopCount_++;
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
    tfLoopCount_++;
    rate.sleep();
  }
  return true;
}

void AsPMPC::setCurrentObservation(const Observation &observation)
{
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
  ROS_WARN_STREAM_THROTTLE(1.0, "initializeCostDesiredTrajectory set");
  trajectoryUpdated_ = true;
}
