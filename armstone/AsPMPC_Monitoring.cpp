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

    int mpcLoopCount;
    int trackerLoopCount;

    {
      boost::unique_lock<boost::shared_mutex> lockGuard1(mpcLoopCountMutex_);
      boost::unique_lock<boost::shared_mutex> lockGuard2(trackerLoopCountMutex_);

      mpcLoopCount = mpcLoopCount_;
      trackerLoopCount = trackerLoopCount_;
      mpcLoopCount_ = 0;
      trackerLoopCount_ = 0;
    }
    double time = ros::Time::now().toSec();
    ROS_INFO_STREAM(std::endl
                    << "    MPC loop rate:     " << std::round(((double)mpcLoopCount) / (time - monitorTimeLast_)) << std::endl
                    << "    tracker loop rate:     " << std::round(((double)trackerLoopCount) / (time - monitorTimeLast_)) << std::endl
                    << std::endl);
    monitorTimeLast_ = time;

    rate.sleep();
  }
  return true;
}