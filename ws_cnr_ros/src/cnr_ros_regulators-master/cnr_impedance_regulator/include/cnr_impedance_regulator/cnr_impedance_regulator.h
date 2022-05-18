#ifndef CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
#define CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H

#include <type_traits>
#include <Eigen/Core>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_state.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace cnr
{
namespace control
{

//!
using BaseImpedanceRegulator = RegulatorInterface<BaseRegulatorParams,
                                                  ImpedanceRegulatorState,
                                                  JointRegulatorReference,
                                                  JointRegulatorControlCommand,
                                                  JointRegulatorFeedback >;

//!
class ImpedanceRegulator : public BaseImpedanceRegulator
{
public:
  typedef std::shared_ptr<ImpedanceRegulator> Ptr;
  typedef std::shared_ptr<ImpedanceRegulator const > ConstPtr;

  ImpedanceRegulator() = default;
  virtual ~ImpedanceRegulator() = default;
  ImpedanceRegulator(const ImpedanceRegulator&) = delete;
  ImpedanceRegulator& operator=(const ImpedanceRegulator&) = delete;
  ImpedanceRegulator(ImpedanceRegulator&&) = delete;
  ImpedanceRegulator& operator=(ImpedanceRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle&   root_nh,
                          ros::NodeHandle&   controller_nh,
                          BaseRegulatorParamsPtr opts) override;

  bool update(BaseRegulatorReferenceConstPtr r,
              BaseRegulatorControlCommandPtr u) override;

  virtual bool starting(BaseRegulatorStateConstPtr state0, const ros::Time& time) override;

  
private:
  rosdyn::MatrixXd m_Jinv;
  rosdyn::MatrixXd m_damping;
  rosdyn::MatrixXd m_damping_dafault;
  rosdyn::MatrixXd m_k;
  rosdyn::MatrixXd m_k_default;
  rosdyn::MatrixXd m_k_new;
};

using ImpedanceRegulatorPtr = typename ImpedanceRegulator::Ptr;
using ImpedanceRegulatorConstPtr = typename ImpedanceRegulator::ConstPtr;


}  // namespace control
}  // namespace cnr

#include <cnr_impedance_regulator/internal/cnr_impedance_regulator_impl.h>

#endif  // CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H
