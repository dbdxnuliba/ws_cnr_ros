#ifndef CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
#define CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H

#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>

namespace cnr
{
namespace control
{

class FakeRegulator: public BaseJointRegulator
{
public:
  typedef std::shared_ptr<FakeRegulator> Ptr;
  typedef std::shared_ptr<FakeRegulator const> ConstPtr;


  FakeRegulator() = default;
  virtual ~FakeRegulator() = default;
  FakeRegulator(const FakeRegulator&) = delete;
  FakeRegulator& operator=(const FakeRegulator&) = delete;
  FakeRegulator(FakeRegulator&&) = delete;
  FakeRegulator& operator=(FakeRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, 
                          BaseRegulatorParamsPtr opts) override;

  virtual bool update(BaseRegulatorReferenceConstPtr r,
                      BaseRegulatorFeedbackConstPtr  y,
                      BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR_THROTTLE(this->logger(),10.0, "The regulator does not need any feedback!");
    return update(r, u);
  }

  bool update(BaseRegulatorFeedbackConstPtr  y,
              BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR(this->logger(), "The regulator needs the reference override, while it does not use the feedback!");
    CNR_RETURN_FALSE(this->logger());
  }

  bool update(BaseRegulatorReferenceConstPtr r,
              BaseRegulatorControlCommandPtr u) override;

  
  bool update(BaseRegulatorControlCommandPtr u) override
  {
    CNR_ERROR(this->logger(), "The regulator needs the reference override!");
    CNR_RETURN_FALSE(this->logger());
  }

  virtual bool starting(BaseRegulatorStateConstPtr state0, const ros::Time& time) override;

};

using FakeRegulatorPtr = typename FakeRegulator::Ptr;
using FakeRegulatorConstPtr = typename FakeRegulator::ConstPtr;


}  // namespace control
}  // namespace cnr


#include <cnr_fake_regulator/internal/cnr_fake_regulator_impl.h>

#endif  // CNR_FAKE_REGULATOR__CNR_FAKE_REGULATOR__H
