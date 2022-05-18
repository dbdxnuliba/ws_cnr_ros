#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_chain_state/chain_state.h>

namespace cnr
{
namespace control
{


/**
 * @brief The BaseRegulatorFeedback struct
 */
struct BaseRegulatorFeedback
{
  typedef std::shared_ptr<BaseRegulatorFeedback> Ptr;
  typedef std::shared_ptr<BaseRegulatorFeedback const> ConstPtr;

  BaseRegulatorFeedback() = default;
  virtual ~BaseRegulatorFeedback() = default;
  BaseRegulatorFeedback(const BaseRegulatorFeedback&) = delete;
  BaseRegulatorFeedback& operator=(const BaseRegulatorFeedback&) = delete;
  BaseRegulatorFeedback(BaseRegulatorFeedback&&) = delete;
  BaseRegulatorFeedback& operator=(BaseRegulatorFeedback&&) = delete;
};

typedef BaseRegulatorFeedback::Ptr BaseRegulatorFeedbackPtr;
typedef BaseRegulatorFeedback::ConstPtr BaseRegulatorFeedbackConstPtr;


/**
 * @brief JointRegulatorFeedback
 */
class JointRegulatorFeedback : public BaseRegulatorFeedback
{
protected:
  rosdyn::ChainState robot_state;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<JointRegulatorFeedback> Ptr;
  typedef std::shared_ptr<JointRegulatorFeedback const> ConstPtr;

  JointRegulatorFeedback() = default;
  virtual ~JointRegulatorFeedback() = default;
  JointRegulatorFeedback(const JointRegulatorFeedback&) = delete;
  JointRegulatorFeedback(JointRegulatorFeedback&&) = delete;
  JointRegulatorFeedback& operator=(JointRegulatorFeedback&&) = delete;

  JointRegulatorFeedback(rosdyn::Chain& kin)
  {
    robot_state.init(kin);
  }

  rosdyn::ChainState& robotState()
  {
    return robot_state;
  }

  const rosdyn::ChainState& robotState() const
  {
    return robot_state;
  }
};

using JointRegulatorFeedbackPtr = typename JointRegulatorFeedback::Ptr;
using JointRegulatorFeedbackConstPtr = typename JointRegulatorFeedback::ConstPtr;


/**
 * @brief The CartesianRegulatorFeedback struct
 */
struct CartesianRegulatorFeedback : public JointRegulatorFeedback
{ 
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<CartesianRegulatorFeedback> Ptr;
  typedef std::shared_ptr<CartesianRegulatorFeedback const> ConstPtr;
  
  virtual ~CartesianRegulatorFeedback() = default;
  CartesianRegulatorFeedback(const CartesianRegulatorFeedback&) = delete;
  
  CartesianRegulatorFeedback(CartesianRegulatorFeedback&&) = delete;
  CartesianRegulatorFeedback& operator=(CartesianRegulatorFeedback&&) = delete;

  CartesianRegulatorFeedback( ) = default;
  CartesianRegulatorFeedback(rosdyn::Chain& kin) : JointRegulatorFeedback(kin)
  {
  }
};

using CartesianRegulatorFeedbackPtr = typename CartesianRegulatorFeedback::Ptr;
using CartesianRegulatorFeedbackConstPtr = typename CartesianRegulatorFeedback::ConstPtr;


}  // namespace control
}  // namespace cnr


#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_FEEDBACK__H
