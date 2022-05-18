#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OPTIONS__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OPTIONS__H

#include <type_traits>
#include <memory>
#include <ros/time.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <rosdyn_chain_state/chain_state.h>

namespace cnr
{
namespace control
{

/**
 * @brief The BaseRegulatorParams struct
 */
struct BaseRegulatorParams
{
  typedef std::shared_ptr<BaseRegulatorParams> Ptr;
  typedef std::shared_ptr<BaseRegulatorParams const> ConstPtr;
  
  BaseRegulatorParams() = default;
  virtual ~BaseRegulatorParams() = default;
  BaseRegulatorParams(const BaseRegulatorParams&) = delete;
  
  BaseRegulatorParams(BaseRegulatorParams&&) = delete;
  BaseRegulatorParams& operator=(BaseRegulatorParams&&) = delete;
  
  BaseRegulatorParams(size_t n_dim) : dim(n_dim){}
  
  void set_dimension(const size_t& n_dim) { dim = n_dim;}
  
  cnr_logger::TraceLoggerPtr  logger;
  size_t                      dim;
  ros::Duration               period;
  std::vector<std::string>    resources_names;
  rosdyn::Chain*              chain;
  InterpolatorBasePtr         interpolator;
  
  BaseRegulatorParams& operator<<(const BaseRegulatorParams::ConstPtr& rhs)
  {
    this->logger = rhs->logger;
    this->dim = rhs->dim;
    this->period = rhs->period;
    this->resources_names = rhs->resources_names;
    this->chain = rhs->chain;
    this->interpolator = rhs->interpolator;
    return *this;
  }
  
  BaseRegulatorParams& operator=(const BaseRegulatorParams& rhs)
  {
    this->logger = rhs.logger;
    this->dim = rhs.dim;
    this->period = rhs.period;
    this->resources_names = rhs.resources_names;
    this->chain = rhs.chain;
    this->interpolator = rhs.interpolator;
    return *this;
  }
  
  //const size_t& nAx() { return robot_kin->nAx(); }
};

typedef BaseRegulatorParams::Ptr BaseRegulatorParamsPtr;
typedef BaseRegulatorParams::ConstPtr BaseRegulatorParamsConstPtr;

struct JointRegulatorParams : public BaseRegulatorParams
{
  JointRegulatorParams() = default;
  virtual ~JointRegulatorParams() = default;
  JointRegulatorParams(const JointRegulatorParams&) = delete;
  JointRegulatorParams& operator=(const JointRegulatorParams&) = delete;
  JointRegulatorParams(JointRegulatorParams&&) = delete;
  JointRegulatorParams& operator=(JointRegulatorParams&&) = delete;
  
  void set_dof(const size_t& n_dim) { set_dimension(n_dim);}
  
};

typedef std::shared_ptr<JointRegulatorParams> JointRegulatorParamsPtr;
typedef std::shared_ptr<JointRegulatorParams const> JointRegulatorParamsConstPtr;


/**
 * @brief The CartesianRegulatorParams struct
 */
struct CartesianRegulatorParams : public BaseRegulatorParams
{
  CartesianRegulatorParams() : BaseRegulatorParams(6) {};
  virtual ~CartesianRegulatorParams() = default;
  CartesianRegulatorParams(const CartesianRegulatorParams&) = delete;
  CartesianRegulatorParams& operator=(const CartesianRegulatorParams&) = delete;
  CartesianRegulatorParams(CartesianRegulatorParams&&) = delete;
  CartesianRegulatorParams& operator=(CartesianRegulatorParams&&) = delete;
};

typedef std::shared_ptr<CartesianRegulatorParams> CartesianRegulatorParamsPtr;
typedef std::shared_ptr<CartesianRegulatorParams const> CartesianRegulatorParamsConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_OPTIONS__H
