#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H

#include <memory>
#include <type_traits>

#include <ros/time.h>
#include <Eigen/Dense>
#include <eigen_matrix_utils/overloads.h>
#include <rosdyn_core/primitives.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

/**
 * @brief The BaseRegulatorControlCommand struct
 */
struct BaseRegulatorControlCommand
{
  using Ptr      = std::shared_ptr<BaseRegulatorControlCommand>;
  using ConstPtr = std::shared_ptr<BaseRegulatorControlCommand const>;

  BaseRegulatorControlCommand() = default;
  virtual ~BaseRegulatorControlCommand() = default;
  BaseRegulatorControlCommand(const BaseRegulatorControlCommand&) = delete;
  BaseRegulatorControlCommand& operator=(const BaseRegulatorControlCommand&) = delete;
  BaseRegulatorControlCommand(BaseRegulatorControlCommand&&) = delete;
  BaseRegulatorControlCommand& operator=(BaseRegulatorControlCommand&&) = delete;

  ros::Duration time_from_start;
  bool in_goal_tolerance;
  bool in_path_tolerance;
  double scaling;
};

using BaseRegulatorControlCommandPtr = typename BaseRegulatorControlCommand::Ptr;
using BaseRegulatorControlCommandConstNPtr = typename BaseRegulatorControlCommand::ConstPtr;

/**
 * @brief The BaseRegulatorControlCommand struct
 */

template<typename Vector>
struct KinematicsRegulatorControlCommand : public BaseRegulatorControlCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<KinematicsRegulatorControlCommand>;
  using ConstPtr = std::shared_ptr<KinematicsRegulatorControlCommand const>;

  Vector x;
  Vector xd;
  Vector xdd;
  Vector eff;

  void set_dimension(const int& dim)
  {
    if(eu::rows(x)==1 && dim!=1) throw std::runtime_error("The command size is a scalar, and it cannot be resized.");
    if(!eu::resize(x,dim))
    {
      throw std::runtime_error("The command is fixed-size, and it cannot be resized.");
    }
    eu::setZero(x);
    eu::resize(xd,dim); eu::setZero(xd);
    eu::resize(xdd,dim); eu::setZero(xdd);
    eu::resize(eff,dim); eu::setZero(eff);
  }

  int dof() const {return eu::rows(x);}

  KinematicsRegulatorControlCommand() = default;
  virtual ~KinematicsRegulatorControlCommand() = default;
  KinematicsRegulatorControlCommand(const KinematicsRegulatorControlCommand&) = delete;
  KinematicsRegulatorControlCommand& operator=(const KinematicsRegulatorControlCommand&) = delete;
  KinematicsRegulatorControlCommand(KinematicsRegulatorControlCommand&&) = delete;
  KinematicsRegulatorControlCommand& operator=(KinematicsRegulatorControlCommand&&) = delete;
};

template <typename Vector>
using KinematicsRegulatorControlCommandPtr = typename KinematicsRegulatorControlCommand<Vector>::Ptr;

template <typename Vector>
using KinematicsRegulatorControlCommandConstPtr = typename KinematicsRegulatorControlCommand<Vector>::ConstPtr;


/**
 * @brief The JointRegulatorControlCommand struct
 */
//!
using JointRegulatorControlCommand = KinematicsRegulatorControlCommand<rosdyn::VectorXd>;

//!
using JointRegulatorControlCommandPtr = KinematicsRegulatorControlCommandPtr<rosdyn::VectorXd>;

//!
using JointRegulatorControlCommandConstPtr = KinematicsRegulatorControlCommandConstPtr<rosdyn::VectorXd>;


/**
 * @brief The CartesianRegulatorControlCommand struct
 */
using CartesianRegulatorControlCommand         = KinematicsRegulatorControlCommand<Eigen::Vector6d>;
using CartesianRegulatorControlCommandPtr      = KinematicsRegulatorControlCommandPtr<Eigen::Vector6d>;
using CartesianRegulatorControlCommandConstPtr = KinematicsRegulatorControlCommandConstPtr<Eigen::Vector6d>;

}  // namespace control
}  // namespace cnr

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OUTPUTS__H
