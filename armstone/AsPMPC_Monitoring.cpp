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
    int tfLoopCount;
    int obsCount;

    {
      boost::unique_lock<boost::shared_mutex> lockGuard1(mpcLoopCountMutex_);
      mpcLoopCount = mpcLoopCount_;
      mpcLoopCount_ = 0;
    }

    {
      boost::unique_lock<boost::shared_mutex> lockGuard2(trackerLoopCountMutex_);
      trackerLoopCount = trackerLoopCount_;
      trackerLoopCount_ = 0;
    }

    {
      boost::unique_lock<boost::shared_mutex> lockGuard3(tfLoopCountMutex_);
      tfLoopCount = tfLoopCount_;
      tfLoopCount_ = 0;
    }

    {
      boost::unique_lock<boost::shared_mutex> lockGuard3(obsCountMutex_);
      obsCount = obsCount_;
      obsCount_ = 0;
    }

    double time = ros::Time::now().toSec();
    double dt = time - monitorTimeLast_;
    ROS_INFO_STREAM(std::endl
                    << "    MPC loop rate:     " << std::round(((double)mpcLoopCount) / dt) << std::endl
                    << "    tracker loop rate: " << std::round(((double)trackerLoopCount) / dt) << std::endl
                    << "    TF loop rate:     " << std::round(((double)tfLoopCount) / dt) << std::endl
                    << "    obs  rate:     " << std::round(((double)obsCount) / dt) << std::endl
                    << std::endl);
    monitorTimeLast_ = time;

    rate.sleep();
  }
  return true;
}