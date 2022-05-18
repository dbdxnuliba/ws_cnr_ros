#pragma once // workaraound qtcreator

#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE_IMPL_H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE_IMPL_H

#include <memory>
#include <ros/node_handle.h>
#include <cnr_logger/cnr_logger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <rosdyn_chain_state/chain_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <cnr_regulator_interface/cnr_regulator_interface.h>

namespace cnr
{
namespace control
{

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::initialize(
    ros::NodeHandle&  root_nh, ros::NodeHandle& controller_nh, BaseRegulatorParamsPtr opts)
{
  if(!BaseRegulator::initialize(root_nh,controller_nh, opts))
  {
    return false;
  }

  CNR_TRACE_START(this->logger());
  const std::shared_ptr<P> _p = std::dynamic_pointer_cast<P>(opts);
  if(!_p)
  {
    CNR_RETURN_TRUE(logger(), "Recast failure. Check the objects type. Abort.");
  }
  CNR_RETURN_TRUE(this->logger());
}

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::starting(BaseRegulatorStateConstPtr x0, const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  std::shared_ptr<X const> _x0 = std::dynamic_pointer_cast<X const>(x0);
  if(!_x0)
  {
    CNR_RETURN_FALSE(logger(), "Recast failure. Check the objects type. Abort.");
  }
  if(!BaseRegulator::starting(_x0, time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  std::shared_ptr<X> x(new X( ));
  *x = *_x0;
  this->x_ = x;
  CNR_RETURN_TRUE(this->logger());
}

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::update(
      BaseRegulatorReferenceConstPtr   r,
      BaseRegulatorFeedbackConstPtr    y,
      BaseRegulatorControlCommandPtr   u)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  auto _r = get(r);
  auto _y = get(y);
  auto _u = get(u);
  if(!_r || !_y || !_u )
  {
    CNR_RETURN_FALSE(this->logger());
  }
  if(!BaseRegulator::update(_r,_y,_u))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::update(BaseRegulatorReferenceConstPtr r, BaseRegulatorControlCommandPtr u)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  auto _r = get(r);
  auto _u = get(u);
  if(!_r || !_u )
  {
    CNR_RETURN_FALSE(this->logger());
  }
  if(!BaseRegulator::update(_r,_u))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::update(BaseRegulatorFeedbackConstPtr   y,
                      BaseRegulatorControlCommandPtr  u)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  auto _y = get(y);
  auto _u = get(u);
  if(!_y || !_u )
  {
    CNR_RETURN_FALSE(this->logger());
  }
  if(!BaseRegulator::update(_y,_u))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::update(BaseRegulatorControlCommandPtr  u)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  auto _u = get(u);
  if(!_u )
  {
    CNR_RETURN_FALSE(this->logger());
  }
  if(!BaseRegulator::update(_u))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class P, class X, class R, class U, class Y>
inline bool RegulatorInterface<P,X,R,U,Y>::stopping(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(!BaseRegulator::stopping(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}


template<class P, class X, class R, class U, class Y>
inline std::shared_ptr<R const > RegulatorInterface<P,X,R,U,Y>::get(BaseRegulatorReferenceConstPtr r)
{
  std::shared_ptr<R const > _r = std::dynamic_pointer_cast<R const>(r);
  if(!_r)
  {
    CNR_ERROR(logger(), "Recast failure. Check the objects type. Abort.");
  }
  return _r;
}


template<class P, class X, class R, class U, class Y>
inline std::shared_ptr<Y const> RegulatorInterface<P,X,R,U,Y>::get(BaseRegulatorFeedbackConstPtr y)
{
  std::shared_ptr<Y const > _y = std::dynamic_pointer_cast<Y const>(y);
  if(!_y)
  {
    CNR_ERROR(logger(), "Recast failure. Check the objects type. Abort.");
  }
  return _y;
}

template<class P, class X, class R, class U, class Y>
inline std::shared_ptr<U>  RegulatorInterface<P,X,R,U,Y>::get(BaseRegulatorControlCommandPtr u)
{
  std::shared_ptr<U> _u = std::dynamic_pointer_cast<U>(u);
  if(!_u)
  {
    CNR_ERROR(logger(), "Recast failure. Check the objects type. Abort.");
  }
  return _u;
}



//!
inline bool BaseJointRegulator::starting(BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(!this->__BaseJointRegulator::starting(state0, time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}



//!
inline bool BaseCartesianRegulator::starting(BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(!RegulatorInterface<CartesianRegulatorParams,CartesianRegulatorState,CartesianRegulatorReference,
                            JointRegulatorControlCommand, JointRegulatorFeedback>::starting(state0, time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

}
}

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_INTERFACE_IMPL_H
