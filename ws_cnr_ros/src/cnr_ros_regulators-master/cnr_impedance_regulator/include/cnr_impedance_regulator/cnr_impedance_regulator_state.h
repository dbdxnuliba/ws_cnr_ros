#ifndef CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H
#define CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_chain_state/chain_state.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_regulator_interface/cnr_regulator_state.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

struct MassSpringDamperModel
{
  rosdyn::VectorXd x;
  rosdyn::VectorXd xd;
  rosdyn::VectorXd xdd;
  rosdyn::VectorXd effort;

  void set_dim(const size_t& dim)
  {
    eu::resize(x     , dim); eu::setZero(x     );
    eu::resize(xd    , dim); eu::setZero(xd    );
    eu::resize(xdd   , dim); eu::setZero(xdd   );
    eu::resize(effort, dim); eu::setZero(effort);
  }

  MassSpringDamperModel& operator=(const MassSpringDamperModel& rhs)
  {
    x = rhs.x;
    xd = rhs.xd;
    xdd = rhs.xdd;
    xdd = rhs.xdd;
    return *this;
  }

  MassSpringDamperModel& operator=(const rosdyn::ChainState& status)
  {
    set_dim(eigen_utils::rows(status.q()));
    x = status.q();
    xd = status.qd();
    xdd = status.qdd();
    effort = status.effort();
    return *this;
  }
};

//!
class ImpedanceRegulatorState : public JointRegulatorState
{
public:
  typedef std::shared_ptr<ImpedanceRegulatorState> Ptr;
  typedef std::shared_ptr<ImpedanceRegulatorState const> ConstPtr;
  
  ImpedanceRegulatorState() = default;
  virtual ~ImpedanceRegulatorState() = default;
  ImpedanceRegulatorState(const ImpedanceRegulatorState&) = delete;
  ImpedanceRegulatorState(ImpedanceRegulatorState&&) = delete;
  ImpedanceRegulatorState& operator=(ImpedanceRegulatorState&&) = delete;
  
  ImpedanceRegulatorState(rosdyn::Chain& kin) : JointRegulatorState(kin) {}
  ImpedanceRegulatorState& operator=(const ImpedanceRegulatorState& rhs)
  {
     msd.set_dim(eu::rows(rhs.msd.x));
     msd = rhs.msd;
     return *this;
  }

  MassSpringDamperModel& msdState()
  {
    return msd;
  }

  const MassSpringDamperModel& msdState() const
  {
    return msd;
  }

protected:
  MassSpringDamperModel msd;

};

//!
using ImpedanceRegulatorStatePtr = typename ImpedanceRegulatorState::Ptr;

//!
using ImpedanceRegulatorStateConstPtr = typename ImpedanceRegulatorState::ConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_STATE__H
