#ifndef REPLANNER_MANAGER_H__
#define REPLANNER_MANAGER_H__

#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <subscription_notifier/subscription_notifier.h>
#include <thread>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Empty.h>
#include <graph_core/solvers/multigoal.h>

namespace pathplan
{
#define K_OFFSET 1.2
class ReplannerManager;
typedef std::shared_ptr<ReplannerManager> ReplannerManagerPtr;

class ReplannerManager: public std::enable_shared_from_this<ReplannerManager>
{
protected:

  // To be assigned by the constructor
  PathPtr current_path_;
  std::vector<PathPtr> other_paths_;
  double trj_exec_thread_frequency_;
  double collision_checker_thread_frequency_;
  double dt_replan_restricted_;
  double dt_replan_relaxed_;
  std::string group_name_;
  std::string base_link_;
  std::string last_link_;
  ros::NodeHandle nh_;

  // Global variables
  bool stop_;
  bool first_replan_;
  bool path_obstructed_;
  bool computing_avoiding_path_;
  bool spawn_objs_;
  bool replanning_;
  bool replan_relaxed_;
  bool display_timing_warning_;
  bool display_replanning_success_;
  bool read_safe_scaling_;
  bool emergency_stop_;
  bool current_path_changed_;

  int n_conn_;
  int pos_closest_obs_from_goal_repl_;
  int pos_closest_obs_from_goal_check_;
  double real_time_;
  double t_;
  double dt_;
  double replan_offset_;
  double t_replan_;
  double replanning_thread_frequency_;
  double scaling_from_param_;
  double scaling_;
  double checker_resol_;

  ReplannerPtr replanner_;
  Eigen::VectorXd current_configuration_;
  Eigen::VectorXd configuration_replan_;
  CollisionCheckerPtr checker_thread_cc_;
  CollisionCheckerPtr checker_;
  TrajectoryPtr trajectory_;
  planning_scene::PlanningScenePtr planning_scn_;
  planning_scene::PlanningScenePtr planning_scn_replanning_;
  trajectory_processing::SplineInterpolator interpolator_;
  trajectory_msgs::JointTrajectoryPoint pnt_;
  trajectory_msgs::JointTrajectoryPoint pnt_replan_;
  sensor_msgs::JointState new_joint_state_;
  robot_state::RobotStatePtr kinematic_state_;

  std::thread display_thread_;
  std::thread spawn_obj_thread_;
  std::thread replanning_thread_;
  std::thread col_check_thread_;

  std::mutex planning_mtx_;
  std::mutex checker_mtx_;
  std::mutex trj_mtx_;
  std::mutex paths_mtx_;
  std::mutex scene_mtx_;
  std::mutex replanner_mtx_;

  std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>> speed_ovr_sub_;
  std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>> safe_ovr_1_sub_;
  std::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>> safe_ovr_2_sub_;
  ros::Publisher current_norm_pub_;
  ros::Publisher new_norm_pub_;
  ros::Publisher time_replanning_pub_;
  ros::Publisher obs_current_norm_pub_;
  ros::Publisher obs_new_norm_pub_;
  ros::Publisher obs_time_replanning_pub_;
  ros::Publisher target_pub_;
  ros::Publisher unscaled_target_pub_;
  ros::Publisher time_pub_;
  ros::ServiceClient plannning_scene_client_;
  ros::ServiceClient add_obj_;
  ros::ServiceClient remove_obj_;
  ros::ServiceClient start_log_;
  ros::ServiceClient stop_log_;
  std_srvs::Empty srv_log_;

  void fromParam();
  void attributeInitialization();
  void subscribeTopicsAndServices();
  void replanningThread();
  void collisionCheckThread();
  void displayThread();
  void spawnObjects();
  bool trajectoryExecutionThread();
  bool sendRobotStateThread();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManager(PathPtr &current_path,
                   std::vector<PathPtr> &other_paths,
                   ros::NodeHandle &nh);

  bool start();
  bool startWithoutReplanning();
};

}

#endif // REPLANNER_MANAGER_H
