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

#pragma once

// perceptive_mpc
#include <armstone/AsPerceptiveMpcInterface.h>
#include <perceptive_mpc/ExplicitTemplateInstantiations.h>
#include <perceptive_mpc/costs/PointsOnRobot.h>

#include <perceptive_mpc/Definitions.h>

#include <m3dp_msgs/TaskTrajectory.h>
#include <m3dp_msgs/PrintTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

// ocs2
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

// ros
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <boost/thread.hpp>

#include <kindr/Core>

#include <perceptive_mpc/EsdfCachingServer.hpp>
// TODO: uncomment for admittance control on hardware:
// #include <perceptive_mpc/AdmittanceReferenceModule.hpp>

namespace perceptive_mpc
{

  class AsPMPC
  {
  public:
    typedef ocs2::SystemObservation<perceptive_mpc::STATE_DIM_, perceptive_mpc::INPUT_DIM_> Observation;
    typedef ocs2::MPC_MRT_Interface<perceptive_mpc::STATE_DIM_, perceptive_mpc::INPUT_DIM_> MpcInterface;
    typedef MpcInterface::input_vector_t InputVector;
    using reference_vector_t = Eigen::Matrix<double, Definitions::REFERENCE_DIM, 1>;

    explicit AsPMPC(const ros::NodeHandle &nh = ros::NodeHandle());

    bool run();

  protected:
    std::string mpcTaskFile_;
    std::unique_ptr<perceptive_mpc::AsPerceptiveMpcInterface> pmpcInterface_;
    std::shared_ptr<MpcInterface> mpcInterface_;
    std::shared_ptr<PointsOnRobot> pointsOnRobot_;
    std::shared_ptr<voxblox::EsdfCachingServer> esdfCachingServer_;

    // params
    std::string end_effector_frame_;
    std::string base_frame_;
    std::string odom_frame_;
    bool sim_mode_;
    bool collision_mode_;
    double mpcUpdateFrequency_;
    double tfUpdateFrequency_;
    double controlLoopFrequency_;
    double maxLinearVelocity_;
    double maxAngularVelocity_;
    double base_mass;
    double deadmanAxes_;

    // flags
    std::atomic_bool planAvailable_;
    std::atomic_bool mpcUpdateFailed_;
    std::atomic_bool firstObservationUpdated_;
    std::atomic_bool trajectoryUpdated_;
    std::atomic_bool mpcEnabled_;

    // safety vars
    double lastDeadManTime_;
    boost::shared_mutex lastDeadManTimeMutex_;
    double lastJointStateTime_;
    boost::shared_mutex lastJointStateTimeMutex_;
    std::atomic_int mpcLoopCount_;
    std::atomic_int trackerLoopCount_;
    std::atomic_int tfLoopCount_;
    std::atomic_int obsCount_;

    std::atomic_int mpcLoopRate_;
    std::atomic_int trackerLoopRate_;
    std::atomic_int tfLoopLoopRate_;
    std::atomic_int obsRate_;

    // Monitor vars
    double monitorTimeLast_;

    // MPC

    MpcInterface::state_vector_t optimalState_;

    boost::shared_mutex observationMutex_;
    Observation observation_;

    MpcInterface::input_vector_t controlInput_;
    boost::shared_mutex controlInputMutex_;

    KinematicInterfaceConfig kinematicInterfaceConfig_;

    boost::shared_mutex costDesiredTrajectoryMutex_;
    ocs2::CostDesiredTrajectories costDesiredTrajectories_; // quaternion of ee, xyz ee, forces? or something

    // ros
    ros::NodeHandle nh_;
    ros::Subscriber goalPoseSubscriber_;
    ros::Subscriber jointStatesSubscriber_;
    ros::Subscriber joySubscriber_;    
    actionlib::SimpleActionServer<m3dp_msgs::PrintTrajectoryAction> taskTrajectoryActionServer_;
    ros::Publisher armJointVelPub_;
    ros::Publisher baseTwistPub_;
    ros::Publisher pointsOnRobotPublisher_;
    ros::Publisher armStatePublisher_;
    ros::Publisher endEffectorPosePublisher_;

    tf2_ros::TransformBroadcaster tfBroadcaster_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    geometry_msgs::Twist baseTwistMsg_;
    std_msgs::Float64MultiArray armJointVelMsg_;

  protected:
    // thread 1 simulates the control loop: query new mpc plan, writes observation
    bool trackerLoop(ros::Rate rate);

    // thread 2 is the mpc solver
    bool mpcUpdate(ros::Rate rate);

    // thread 3  for sim tf publish
    bool tfUpdate(ros::Rate rate);

    // thread 4  for loop monitoring
    bool loopMonitor(ros::Rate rate);

    //Member thread
    std::thread *tfUpdateWorker_;

    // compute the current end effector Pose on the base of the latest observation
    kindr::HomTransformQuatD getEndEffectorPose();
    kindr::HomTransformQuatD getBasePose();

    // parse all ros parameters
    void parseParameters();

    std::shared_ptr<VoxbloxCostConfig> configureCollisionAvoidance(std::shared_ptr<KinematicsInterfaceAD> kinematicInterface);

    // update the desired end effector pose on ros msg
    void desiredEndEffectorPoseCb(const geometry_msgs::PoseStampedConstPtr &msgPtr);

    void joyCb(const sensor_msgs::JoyPtr &msgPtr);

    bool isDead(); //checks if joy msg was old and or joint_states is old

    void setTaskTrajectory(const m3dp_msgs::TaskTrajectory &taskTrajectory);

    void printTrajectoryActionCb(const m3dp_msgs::PrintTrajectoryGoalConstPtr &goal);

    // update joint_state
    void jointStatesCb(const sensor_msgs::JointStateConstPtr &msgPtr);

    void pubControlInput(const MpcInterface::input_vector_t &controlInput);
    void pubControlInputZero();

    // make sure the forwarded integrated state is normalized to unit quaternion observation for the base rotation
    void setCurrentObservation(const Observation &observation);

    void loadTransforms();

    // publish the transform from odom to the robot base
    void publishBaseTransform(const Observation &observation);

    // publish the joint state message of the arm state
    void publishArmState(const Observation &observation);

    // publish the current end effector pose to ros
    void publishEndEffectorPose();

    void initializeCostDesiredTrajectory();
  };
} // namespace perceptive_mpc