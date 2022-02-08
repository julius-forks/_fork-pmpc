#include <geometry_msgs/TransformStamped.h>

#include <armstone/AsPMPC.h>

#include <perceptive_mpc/kinematics/asArm/asArmKinematics.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <perceptive_mpc/costs/VoxbloxCost.h>

#include <perceptive_mpc/costs/InterpolatePoseTrajectory.h>

#include <kindr_ros/kindr_ros.hpp>

using namespace perceptive_mpc;

void AsPMPC::printTrajectoryActionCb(const m3dp_msgs::PrintTrajectoryGoalConstPtr &goal)
{
  if (isDead_)
  {
    ROS_WARN("Print Trajectory called while isDead");
    return;
  }
  ROS_WARN("MPC Action Server received a new trajectory task");

  ros::Rate r(5);
  auto feedback = m3dp_msgs::PrintTrajectoryFeedback();
  auto result = m3dp_msgs::PrintTrajectoryResult();
  setTaskTrajectory(goal->trajectory);

  double start_time;
  double end_time;
  {
    boost::shared_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
    start_time = costDesiredTrajectories_.desiredTimeTrajectory()[2]; //Note I'm ignoring first point
    end_time = costDesiredTrajectories_.desiredTimeTrajectory().back();
  }

  ROS_WARN_STREAM(std::endl
                  << "    Start time:          " << start_time << std::endl
                  << "    end_time:          " << end_time << std::endl
                  << std::endl);

  while (ros::ok())
  {
    // check that preempt has not been requested by the client
    if (taskTrajectoryActionServer_.isPreemptRequested())
    {
      ROS_WARN("PrintTrajectory: Preempted");
      taskTrajectoryActionServer_.setPreempted();
      result.success = false;
      break;
    }

    if (isDead_)
    {
      ROS_WARN("PrintTrajectory: isDead detected");
      taskTrajectoryActionServer_.setPreempted();
      result.success = false;
      break;
    }

    if (ros::Time::now().toSec() > end_time + 1)
    {
      ROS_WARN("PrintTrajectory: Completed");
      result.success = true;
      taskTrajectoryActionServer_.setSucceeded(result);
      break;
    }

    feedback.mpc_rate = mpcLoopRate_;
    feedback.tracker_rate = trackerLoopRate_;
    feedback.tf_rate = tfLoopLoopRate_;
    feedback.obs_rate = mpcLoopRate_;
    feedback.completion = (ros::Time::now().toSec() - start_time) / (end_time - start_time);
    taskTrajectoryActionServer_.publishFeedback(feedback);
    r.sleep();
  }
  reinitMpc_ = true;
  ROS_WARN("PrintTrajectory is done.");
}

void AsPMPC::setTaskTrajectory(const m3dp_msgs::TaskTrajectory &taskTrajectory)
{

  reinitMpc_ = false;
  while (ros::ok() && trajectoryUpdated_)
  {
    ros::Rate(100).sleep();
  }

  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  int N = taskTrajectory.points.size(); // point count
  costDesiredTrajectories_.desiredStateTrajectory().resize(N + 3);
  costDesiredTrajectories_.desiredTimeTrajectory().resize(N + 3);
  costDesiredTrajectories_.desiredInputTrajectory().resize(N + 3);

  // Get current
  auto currentBasePose = getBasePose();
  auto currentEEPose = getEndEffectorPose();
  reference_vector_t reference0;
  reference0.head<Definitions::POSE_DIM>().head<4>() = currentEEPose.getRotation().toImplementation().coeffs();
  reference0.head<Definitions::POSE_DIM>().tail<3>() = currentEEPose.getPosition().toImplementation();
  reference0.segment<7>(Definitions::POSE_DIM).head<4>() = currentBasePose.getRotation().getUnique().toImplementation().coeffs();
  reference0.segment<7>(Definitions::POSE_DIM).tail<3>() = currentBasePose.getPosition().toImplementation();
  reference0.tail<3>().head<1>() = Eigen::Matrix<double, 1, 1>(0.1); //+- 10cm
  reference0.tail<2>().head<1>() = Eigen::Matrix<double, 1, 1>(0.1); //+- 10cm
  reference0.tail<1>() = Eigen::Matrix<double, 1, 1>(0.1);           //+- 5deg

  for (int i = 0; i < N + 3; i++)
  {
    if (i == 0)
    {                                                                    //Current Now
      costDesiredTrajectories_.desiredStateTrajectory()[i] = reference0; //shove into desire STATE trajecotry
      costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = ros::Time::now().toSec();
    }
    else if (i == 1)
    {
      //Current in 1 sec
      costDesiredTrajectories_.desiredStateTrajectory()[i] = reference0; //shove into desire STATE trajecotry
      costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = costDesiredTrajectories_.desiredTimeTrajectory()[0] + 1;
    }
    else if (i == 2)
    {
      //trajectory start in 10 seconds
      int j = 0;
      kindr::HomTransformQuatD desiredEEPose;
      kindr::HomTransformQuatD desiredBPose;
      reference_vector_t reference;
      kindr_ros::convertFromRosGeometryMsg(taskTrajectory.points[j].ee_pose, desiredEEPose);  // to kindr pose
      kindr_ros::convertFromRosGeometryMsg(taskTrajectory.points[j].base_pose, desiredBPose); // to kindr pose
      reference.head<Definitions::POSE_DIM>().head<4>() = desiredEEPose.getRotation().toImplementation().coeffs();
      reference.head<Definitions::POSE_DIM>().tail<3>() = desiredEEPose.getPosition().toImplementation();
      reference.segment<7>(Definitions::POSE_DIM).head<4>() = desiredBPose.getRotation().toImplementation().coeffs();
      reference.segment<7>(Definitions::POSE_DIM).tail<3>() = desiredBPose.getPosition().toImplementation();
      reference.tail<3>().head<1>() = Eigen::Matrix<double, 1, 1>(taskTrajectory.points[j].tol_elipse.x_tol);
      reference.tail<2>().head<1>() = Eigen::Matrix<double, 1, 1>(taskTrajectory.points[j].tol_elipse.y_tol);
      reference.tail<1>() = Eigen::Matrix<double, 1, 1>(taskTrajectory.points[j].tol_elipse.th_tol);

      costDesiredTrajectories_.desiredStateTrajectory()[i] = reference; //shove into desire STATE trajecotry
      costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = costDesiredTrajectories_.desiredTimeTrajectory()[1] + 10.0;
    }
    else
    {
      //Start moving at 11.5 seconds
      int j = i - 3;
      kindr::HomTransformQuatD desiredEEPose;
      kindr::HomTransformQuatD desiredBPose;
      reference_vector_t reference;
      kindr_ros::convertFromRosGeometryMsg(taskTrajectory.points[j].ee_pose, desiredEEPose);  // to kindr pose
      kindr_ros::convertFromRosGeometryMsg(taskTrajectory.points[j].base_pose, desiredBPose); // to kindr pose
      reference.head<Definitions::POSE_DIM>().head<4>() = desiredEEPose.getRotation().toImplementation().coeffs();
      reference.head<Definitions::POSE_DIM>().tail<3>() = desiredEEPose.getPosition().toImplementation();
      reference.segment<7>(Definitions::POSE_DIM).head<4>() = desiredBPose.getRotation().toImplementation().coeffs();
      reference.segment<7>(Definitions::POSE_DIM).tail<3>() = desiredBPose.getPosition().toImplementation();
      reference.tail<3>().head<1>() = Eigen::Matrix<double, 1, 1>(taskTrajectory.points[j].tol_elipse.x_tol);
      reference.tail<2>().head<1>() = Eigen::Matrix<double, 1, 1>(taskTrajectory.points[j].tol_elipse.y_tol);
      reference.tail<1>() = Eigen::Matrix<double, 1, 1>(taskTrajectory.points[j].tol_elipse.th_tol);

      costDesiredTrajectories_.desiredStateTrajectory()[i] = reference; //shove into desire STATE trajecotry
      costDesiredTrajectories_.desiredInputTrajectory()[i] = MpcInterface::input_vector_t::Zero();
      costDesiredTrajectories_.desiredTimeTrajectory()[i] = costDesiredTrajectories_.desiredTimeTrajectory()[2] + 1.5 + taskTrajectory.points[j].time_from_start.toSec();
    }
  }
  sendDesiredTrajectory(); //send out the diagnostics
  trajectoryUpdated_ = true;
}

void AsPMPC::sendDesiredTrajectory()
{
  {
    boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
    int N = costDesiredTrajectories_.desiredStateTrajectory().size();
    m3dp_msgs::TaskTrajectory taskTrajectory = m3dp_msgs::TaskTrajectory();
    taskTrajectory.points.resize(N);

    for (int i = 0; i < N; i++)
    {
      m3dp_msgs::TaskPoint point = m3dp_msgs::TaskPoint();
      point.ee_pose.position.x = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).tail<3>()[0];
      point.ee_pose.position.y = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).tail<3>()[1];
      point.ee_pose.position.z = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).tail<3>()[2];
      point.ee_pose.orientation.x = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).head<4>()[0];
      point.ee_pose.orientation.y = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).head<4>()[1];
      point.ee_pose.orientation.z = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).head<4>()[2];
      point.ee_pose.orientation.w = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(0).head<4>()[3];

      point.base_pose.position.x = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).tail<3>()[0];
      point.base_pose.position.y = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).tail<3>()[1];
      point.base_pose.position.z = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).tail<3>()[2];
      point.base_pose.orientation.x = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).head<4>()[0];
      point.base_pose.orientation.y = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).head<4>()[1];
      point.base_pose.orientation.z = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).head<4>()[2];
      point.base_pose.orientation.w = costDesiredTrajectories_.desiredStateTrajectory()[i].segment<7>(7).head<4>()[3];

      point.tol_elipse.x_tol = costDesiredTrajectories_.desiredStateTrajectory()[i].tail<3>()[0];
      point.tol_elipse.y_tol = costDesiredTrajectories_.desiredStateTrajectory()[i].tail<3>()[1];
      point.tol_elipse.th_tol = costDesiredTrajectories_.desiredStateTrajectory()[i].tail<3>()[2];

      point.abs_time = ros::Time(costDesiredTrajectories_.desiredTimeTrajectory()[i]);
      taskTrajectory.points.push_back(point);
    }
    desirecTrajectoryPublisher_.publish(taskTrajectory);
  }
}

void AsPMPC::desiredEndEffectorPoseCb(const geometry_msgs::PoseStampedConstPtr &msgPtr)
{
  if (isDead_)
  {
    ROS_WARN("Desired pose set while is dead");
    return;
  }

  reinitMpc_ = false;
  while (ros::ok() && trajectoryUpdated_)
  {
    ros::Rate(100).sleep();
  }

  geometry_msgs::Pose currentEndEffectorPose;
  kindr_ros::convertToRosGeometryMsg(getEndEffectorPose(), currentEndEffectorPose);
  kindr::HomTransformQuatD desiredPose;
  kindr::HomTransformQuatD currentPose;
  kindr_ros::convertFromRosGeometryMsg(msgPtr->pose, desiredPose);
  kindr_ros::convertFromRosGeometryMsg(currentEndEffectorPose, currentPose);
  auto currentBasePose = getBasePose();

  boost::unique_lock<boost::shared_mutex> costDesiredTrajectoryLock(costDesiredTrajectoryMutex_);
  costDesiredTrajectories_.clear();
  int N = 2;
  costDesiredTrajectories_.desiredStateTrajectory().resize(N);
  costDesiredTrajectories_.desiredTimeTrajectory().resize(N);
  costDesiredTrajectories_.desiredInputTrajectory().resize(N);

  reference_vector_t reference0;
  reference0.head<Definitions::POSE_DIM>().head<4>() = currentPose.getRotation().toImplementation().coeffs();
  reference0.head<Definitions::POSE_DIM>().tail<3>() = currentPose.getPosition().toImplementation();
  reference0.segment<7>(Definitions::POSE_DIM).head<4>() = currentBasePose.getRotation().getUnique().toImplementation().coeffs();
  reference0.segment<7>(Definitions::POSE_DIM).tail<3>() = currentBasePose.getPosition().toImplementation();

  reference_vector_t reference1;
  reference1.head<Definitions::POSE_DIM>().head<4>() = desiredPose.getRotation().toImplementation().coeffs();
  reference1.head<Definitions::POSE_DIM>().tail<3>() = desiredPose.getPosition().toImplementation();
  reference1.segment<7>(Definitions::POSE_DIM).head<4>() = currentBasePose.getRotation().getUnique().toImplementation().coeffs();
  reference1.segment<7>(Definitions::POSE_DIM).tail<3>() = currentBasePose.getPosition().toImplementation();

  costDesiredTrajectories_.desiredStateTrajectory()[0] = reference0;
  costDesiredTrajectories_.desiredStateTrajectory()[1] = reference1;
  costDesiredTrajectories_.desiredInputTrajectory()[0] = MpcInterface::input_vector_t::Zero();
  costDesiredTrajectories_.desiredInputTrajectory()[1] = MpcInterface::input_vector_t::Zero();

  auto minTimeLinear = (desiredPose.getPosition() - currentPose.getPosition()).norm() / maxLinearVelocity_;
  auto minTimeAngular = std::abs(desiredPose.getRotation().getDisparityAngle(currentPose.getRotation())) / maxAngularVelocity_;

  double segmentDuration = std::max(minTimeLinear, minTimeAngular) + 2;

  costDesiredTrajectories_.desiredTimeTrajectory()[0] = ros::Time::now().toSec();
  costDesiredTrajectories_.desiredTimeTrajectory()[1] = costDesiredTrajectories_.desiredTimeTrajectory()[0] + segmentDuration;

  trajectoryUpdated_ = true;
}

void AsPMPC::publishBaseTransform(const Observation &observation)
{
  geometry_msgs::TransformStamped base_transform;
  base_transform.header.frame_id = odom_frame_;
  base_transform.child_frame_id = base_frame_;

  Eigen::Quaterniond currentRotation = Eigen::Quaterniond(observation.state().head<Definitions::BASE_STATE_DIM_>().head<4>());
  currentRotation.normalize();
  const Eigen::Matrix<double, 3, 1> currentPosition = observation.state().head<Definitions::BASE_STATE_DIM_>().tail<3>();

  base_transform.transform.translation.x = currentPosition(0);
  base_transform.transform.translation.y = currentPosition(1);
  base_transform.transform.translation.z = currentPosition(2);

  base_transform.transform.rotation.x = currentRotation.coeffs()(0);
  base_transform.transform.rotation.y = currentRotation.coeffs()(1);
  base_transform.transform.rotation.z = currentRotation.coeffs()(2);
  base_transform.transform.rotation.w = currentRotation.coeffs()(3);

  base_transform.header.stamp = ros::Time::now();
  tfBroadcaster_.sendTransform(base_transform);
}

void AsPMPC::publishArmState(const Observation &observation)
{
  sensor_msgs::JointState armState;
  armState.header.stamp = ros::Time::now();
  armState.name = {"xarmjoint1", "xarmjoint2", "xarmjoint3", "xarmjoint4", "xarmjoint5", "xarmjoint6"};
  armState.position.resize(Definitions::ARM_STATE_DIM_);
  Eigen::VectorXd armConfiguration = observation.state().tail<6>();
  for (size_t joint_idx = 0; joint_idx < Definitions::ARM_STATE_DIM_; joint_idx++)
  {
    armState.position[joint_idx] = armConfiguration(joint_idx);
  }
  armStatePublisher_.publish(armState);
}

void AsPMPC::publishEndEffectorPose()
{
  geometry_msgs::PoseStamped endEffectorPoseMsg;
  static int endEffectorPoseCounter = 0;
  auto currentEndEffectorPose = getEndEffectorPose();
  Eigen::Vector3d currentPosition = currentEndEffectorPose.getPosition().toImplementation();
  Eigen::Quaterniond currentRotation = currentEndEffectorPose.getRotation().getUnique().toImplementation();

  // fill msg
  endEffectorPoseMsg.header.stamp = ros::Time::now();
  endEffectorPoseMsg.header.frame_id = odom_frame_;
  endEffectorPoseMsg.header.seq = endEffectorPoseCounter++;
  endEffectorPoseMsg.pose.position.x = currentPosition(0);
  endEffectorPoseMsg.pose.position.y = currentPosition(1);
  endEffectorPoseMsg.pose.position.z = currentPosition(2);
  endEffectorPoseMsg.pose.orientation.x = currentRotation.coeffs()(0);
  endEffectorPoseMsg.pose.orientation.y = currentRotation.coeffs()(1);
  endEffectorPoseMsg.pose.orientation.z = currentRotation.coeffs()(2);
  endEffectorPoseMsg.pose.orientation.w = currentRotation.coeffs()(3);
  endEffectorPosePublisher_.publish(endEffectorPoseMsg);
}

void AsPMPC::jointStatesCb(const sensor_msgs::JointStateConstPtr &msgPtr)
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
  }
  {
    boost::unique_lock<boost::shared_mutex> lock(observationMutex_);
    boost::unique_lock<boost::shared_mutex> lock2(lastJointStateTimeMutex_);
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
    observation_.time() = ros::Time::now().toSec();
    lastJointStateTime_ = ros::Time::now().toSec();
  }
  // TODO put FIR 0.6 0.35 0.05. Need to copy over things.

  firstObservationUpdated_ = true;
  obsCount_++;
}

void AsPMPC::joyCb(const sensor_msgs::JoyPtr &msgPtr)
{
  //Pressed is -1. Unpressed is 1. But it starts at 0. So if I keep trigger pressed on launch it stays zero. release leass to 1.
  // Axes 6 should be left pad on all joysticks
  {
    boost::unique_lock<boost::shared_mutex> lockGuard(lastDeadManTimeMutex_); //write mutex
    if (msgPtr->axes[deadmanAxes_] > 0.9)
    {
      lastDeadManTime_ = ros::Time::now().toSec();
    }
  }
}

void AsPMPC::pubControlInput(const MpcInterface::input_vector_t &controlInput)
{
  if (isDead_)
  {
    pubControlInputZero();
    return;
  }

  baseTwistMsg_.linear.x = std::max(std::min(controlInput[0], 0.2), -0.2);
  baseTwistMsg_.linear.y = std::max(std::min(controlInput[1], 0.2), -0.2);
  baseTwistMsg_.angular.z = std::max(std::min(controlInput[2], 0.5), -0.5);

  armJointVelMsg_.data.clear();
  for (int i = 3; i < 8; i++)
  {
    armJointVelMsg_.data.push_back(std::max(std::min(controlInput[i], 1.), -1.));
  }
  armJointVelMsg_.data.push_back(0.0); //Last joint always 0.
  baseTwistPub_.publish(baseTwistMsg_);
  armJointVelPub_.publish(armJointVelMsg_);
}

void AsPMPC::pubControlInputZero()
{

  baseTwistMsg_.linear.x = 0.0;
  baseTwistMsg_.linear.y = 0.0;
  baseTwistMsg_.angular.z = 0.0;

  armJointVelMsg_.data.clear();
  for (int i = 3; i < 9; i++)
  {
    armJointVelMsg_.data.push_back(0.0);
  }
  baseTwistPub_.publish(baseTwistMsg_);
  armJointVelPub_.publish(armJointVelMsg_);
}