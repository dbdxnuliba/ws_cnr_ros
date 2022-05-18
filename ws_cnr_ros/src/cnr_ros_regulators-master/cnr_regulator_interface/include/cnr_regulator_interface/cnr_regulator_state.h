#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_chain_state/chain_state.h>

namespace cnr
{
namespace control
{

/**
 * @brief The BaseRegulatorState struct
 */
struct BaseRegulatorState
{
  BaseRegulatorState() = default;
  virtual ~BaseRegulatorState() = default;
  BaseRegulatorState(const BaseRegulatorState&) = delete;
  BaseRegulatorState& operator=(const BaseRegulatorState&) = default;
  BaseRegulatorState(BaseRegulatorState&&) = delete;
  BaseRegulatorState& operator=(BaseRegulatorState&&) = delete;
};

typedef std::shared_ptr<BaseRegulatorState> BaseRegulatorStatePtr;
typedef std::shared_ptr<BaseRegulatorState const> BaseRegulatorStateConstPtr;

/**
 * @brief JointRegulatorState
 */
class JointRegulatorState : public BaseRegulatorState
{
protected:
  rosdyn::ChainState robot_state;

public:
  typedef std::shared_ptr<JointRegulatorState> Ptr;
  typedef std::shared_ptr<JointRegulatorState const> ConstPtr;

  JointRegulatorState() = default;
  virtual ~JointRegulatorState() = default;
  JointRegulatorState(const JointRegulatorState& cpy)
  {
    *this = cpy;
  }

  JointRegulatorState(JointRegulatorState&&) = delete;
  JointRegulatorState& operator=(JointRegulatorState&&) = delete;

  JointRegulatorState(rosdyn::Chain& kin)
    : robot_state(kin)
  {
  }

  //! no update transform!
  JointRegulatorState& operator=(const JointRegulatorState& rhs)
  {
    robot_state.copy(rhs.robot_state, rhs.robot_state.ONLY_JOINT);
    return *this;
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

using JointRegulatorStatePtr = typename JointRegulatorState::Ptr;
using JointRegulatorStateConstPtr = typename JointRegulatorState::ConstPtr;


/**
 * @brief The CartesianRegulatorState struct
 */
using CartesianRegulatorState = JointRegulatorState;
using CartesianRegulatorStatePtr = typename CartesianRegulatorState::Ptr;
using CartesianRegulatorStateConstPtr = typename CartesianRegulatorState::ConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_STATE__H
