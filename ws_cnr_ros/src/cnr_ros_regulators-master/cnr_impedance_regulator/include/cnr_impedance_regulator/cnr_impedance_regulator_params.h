#ifndef CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H
#define CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H

#include <memory>
#include <Eigen/Dense>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_chain_state/chain_state.h>
#include <cnr_regulator_interface/cnr_regulator_params.h>

namespace cnr
{
namespace control
{

struct ImpedanceRegulatorParams : public BaseRegulatorParams
{
  typedef std::shared_ptr<ImpedanceRegulatorParams> Ptr;
  typedef std::shared_ptr<ImpedanceRegulatorParams const> ConstPtr;
  
  ImpedanceRegulatorParams() = default;
  virtual ~ImpedanceRegulatorParams() = default;
  ImpedanceRegulatorParams(const ImpedanceRegulatorParams&) = delete;
  ImpedanceRegulatorParams(ImpedanceRegulatorParams&&) = delete;
  
  ImpedanceRegulatorParams& operator=(ImpedanceRegulatorParams&&) = delete;
  ImpedanceRegulatorParams(const size_t nAx) : BaseRegulatorParams(nAx) {};
  
};

typedef ImpedanceRegulatorParams::Ptr ImpedanceRegulatorParamsPtr;
typedef ImpedanceRegulatorParams::ConstPtr ImpedanceRegulatorParamsConstPtr;


}  // namespace control
}  // namespace cnr

#endif  // CNR_IMPEDANCE_REGULATOR_INTERFACE__CNR_IMPEDANCE_REGULATOR_OPTIONS__H
