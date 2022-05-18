
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include <name_sorting/sort_trajectories.h>

// ----

namespace trajectory_processing
{
bool sort_trajectory(const std::vector<std::string>& joint_names, const trajectory_msgs::JointTrajectory& trj, trajectory_msgs::JointTrajectory& sorted_trj)
{
  const std::vector<std::string>& names=trj.joint_names;
  if (names.size()!=joint_names.size())
  {
    ROS_ERROR("Joint names dimensions are different");
    return false;
  }
  std::vector<int> order_idx(joint_names.size());



  for (unsigned int iOrder=0;iOrder<joint_names.size();iOrder++)
  {
    ROS_DEBUG("index %u, original trajectory %s, sorted trajectory %s",iOrder,names.at(iOrder).c_str(),joint_names.at(iOrder).c_str());
    if (names.at(iOrder).compare(joint_names.at(iOrder)))
    {
      for (unsigned int iNames=0;iNames<names.size();iNames++)
      {
        if (!joint_names.at(iOrder).compare(names.at(iNames)))
        {
          order_idx.at(iOrder)=iNames;
          ROS_DEBUG("Joint %s (index %u) of original trajectory will be in position %u",names.at(iNames).c_str(),iOrder,iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          ROS_ERROR("Joint %s missing",joint_names.at(iOrder).c_str());
          return false;
        }
      }
    }
    else
    {
      order_idx.at(iOrder)=iOrder;
      ROS_DEBUG("Joint %s (index %u) of original trajectory will be in position %u",names.at(iOrder).c_str(),iOrder,iOrder);
    }
  }

  sorted_trj.joint_names=joint_names;
  sorted_trj.header=trj.header;

  for (const trajectory_msgs::JointTrajectoryPoint& pnt: trj.points)
  {
    sorted_trj.points.push_back(pnt);
    for (unsigned int iOrder=0;iOrder<joint_names.size();iOrder++)
    {
      sorted_trj.points.back().positions.at(iOrder)=pnt.positions.at(order_idx.at(iOrder));
      if (pnt.velocities.size()>0)
        sorted_trj.points.back().velocities.at(iOrder)=pnt.velocities.at(order_idx.at(iOrder));
      if (pnt.accelerations.size()>0)
        sorted_trj.points.back().accelerations.at(iOrder)=pnt.accelerations.at(order_idx.at(iOrder));
      if (pnt.effort.size()>0)
        sorted_trj.points.back().effort.at(iOrder)=pnt.effort.at(order_idx.at(iOrder));
    }
  }
  return true;
}


bool append_trajectories(trajectory_msgs::JointTrajectory& trj, const trajectory_msgs::JointTrajectory& trj_to_be_appended)
{
  trajectory_msgs::JointTrajectory sorted_trj_to_be_appended;
  if (!sort_trajectory(trj.joint_names,trj_to_be_appended,sorted_trj_to_be_appended))
    return false;


  ros::Duration end_of_original_trj=trj.points.back().time_from_start+ros::Duration(0.01);

  for (const trajectory_msgs::JointTrajectoryPoint& pnt: sorted_trj_to_be_appended.points)
  {
    trj.points.push_back(pnt);
    trj.points.back().time_from_start=trj.points.back().time_from_start+end_of_original_trj;
  }

  return true;
}

void removeDuplicates(trajectory_msgs::JointTrajectory& trj)
{
  for (unsigned int iPnt=1;iPnt<trj.points.size();iPnt++)
  {
    if ((trj.points.at(iPnt).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec())>1e-6)
    {
      break;
    }

    for (unsigned int iax=0;iax<trj.points.at(iPnt).positions.size();iax++)
    {
      if (std::abs(trj.points.at(iPnt).positions.at(iax)-trj.points.at(iPnt-1).positions.at(iax))>1e-4)
      {
        break;
      }
    }

    ROS_FATAL("erasing point %u",iPnt);
    trj.points.erase(trj.points.begin()+iPnt);
  }
  return;
};

bool computeAccelerationVelocity(trajectory_msgs::JointTrajectory& trj)
{
  if (trj.points.size()<2)
  {
    ROS_ERROR("trajectory should have at least 2 points");
    return false;
  }

  for (unsigned int iPnt=1;iPnt<trj.points.size();iPnt++)
  {
    trj.points.at(iPnt).velocities.resize(trj.points.at(iPnt).positions.size(),0);
    trj.points.at(iPnt).accelerations.resize(trj.points.at(iPnt).accelerations.size(),0);
  }

  for (unsigned int iPnt=1;iPnt<(trj.points.size());iPnt++)
    trj.points.at(iPnt).time_from_start=ros::Duration(trj.points.at(iPnt-1).time_from_start.toSec()+std::max(1.0e-5,trj.points.at(iPnt).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec()));

  for  (unsigned int iAx=0;iAx<trj.points.at(0).positions.size();iAx++)
  {

    for (unsigned int iPnt=1;iPnt<(trj.points.size()-1);iPnt++)
      trj.points.at(iPnt).velocities.at(iAx)=(trj.points.at(iPnt+1).positions.at(iAx)-trj.points.at(iPnt-1).positions.at(iAx))/(trj.points.at(iPnt+1).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec());

    //compute accelerations
    for (unsigned int iPnt=1;iPnt<(trj.points.size()-1);iPnt++)
      trj.points.at(iPnt).accelerations.at(iAx)=(trj.points.at(iPnt+1).velocities.at(iAx)-trj.points.at(iPnt-1).velocities.at(iAx))/(trj.points.at(iPnt+1).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec());

  }

  return true;
}

bool computeAccelerationVelocitySpline(trajectory_msgs::JointTrajectory& trj)
{

  if (trj.points.size()<2)
  {
    ROS_ERROR("trajectory should have at least 2 points");
    return false;
  }


  for (unsigned int iPnt=1;iPnt<trj.points.size();iPnt++)
  {
    trj.points.at(iPnt).velocities.resize(trj.points.at(iPnt).positions.size(),0);
    trj.points.at(iPnt).accelerations.resize(trj.points.at(iPnt).accelerations.size(),0);
  }

  for (unsigned int iPnt=1;iPnt<(trj.points.size());iPnt++)
    trj.points.at(iPnt).time_from_start = ros::Duration(trj.points.at(iPnt-1).time_from_start.toSec()
                                        + std::max(1.0e-5
                                                  , trj.points.at(iPnt).time_from_start.toSec()-trj.points.at(iPnt-1).time_from_start.toSec()));

  for  (unsigned int iAx=0;iAx<trj.points.at(0).positions.size();iAx++)
  {
    // p(t) = p(0)+v*t + 0.5*a*t^2
    // p(0) = p(0)
    // p(t1) = p(0)+v*t1+0.5*a*(t1^2)
    // p(t2) = p(0)+v*t2+0.5*a*(t2^2)

//    [t1 0.5*t1^2]  [v] = [p(t1)-p(0)]
//    [t2 0.5*t2^2]  [a] = [p(t2)-p(0)]

    for (unsigned int iPnt=1;iPnt<(trj.points.size()-1);iPnt++)
    {
      double t1=trj.points.at(iPnt-1).time_from_start.toSec()-trj.points.at(iPnt).time_from_start.toSec();
      double t2=trj.points.at(iPnt+1).time_from_start.toSec()-trj.points.at(iPnt).time_from_start.toSec();

      double dp1=trj.points.at(iPnt-1).positions.at(iAx)-trj.points.at(iPnt).positions.at(iAx);
      double dp2=trj.points.at(iPnt+1).positions.at(iAx)-trj.points.at(iPnt).positions.at(iAx);

      Eigen::MatrixXd mtx(2,2);
      mtx(0,0)= t1; mtx(0,1)=0.5*t1*t1;
      mtx(1,0)= t2; mtx(1,1)=0.5*t2*t2;
      Eigen::VectorXd dp(2);
      dp(0)=dp1;
      dp(1)=dp2;

      Eigen::VectorXd res=mtx.inverse()*dp;
      trj.points.at(iPnt-1).velocities.at(iAx)=res(0);
      trj.points.at(iPnt-1).accelerations.at(iAx)=res(1);

    }

  }

  return true;
}
trajectory_msgs::JointTrajectory createDenseTrajectory(const trajectory_msgs::JointTrajectory& trj, double sampling_period);
}



