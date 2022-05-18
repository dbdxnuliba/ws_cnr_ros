#ifndef NAME_SORTING__SORT_TRAJECTORIES_H
#define NAME_SORTING__SORT_TRAJECTORIES_H

// ----
#include <trajectory_msgs/JointTrajectory.h>

namespace trajectory_processing
{

bool sort_trajectory(const std::vector<std::string>& joint_names,
                      const trajectory_msgs::JointTrajectory& trj,
                        trajectory_msgs::JointTrajectory& sorted_trj);

bool append_trajectories(trajectory_msgs::JointTrajectory& trj,
                          const trajectory_msgs::JointTrajectory& trj_to_be_appended);

void removeDuplicates(trajectory_msgs::JointTrajectory& trj);

bool computeAccelerationVelocity(trajectory_msgs::JointTrajectory& trj);

bool computeAccelerationVelocitySpline(trajectory_msgs::JointTrajectory& trj);


#if 0 
// ???
trajectory_msgs::JointTrajectory createDenseTrajectory(const trajectory_msgs::JointTrajectory& trj, double sampling_period);
// ???
#endif

}  // namespace trajectory_processing

#endif // NAME_SORTING__SORT_TRAJECTORIES_H
