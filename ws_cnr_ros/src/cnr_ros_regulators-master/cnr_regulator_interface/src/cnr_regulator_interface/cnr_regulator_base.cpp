#include <cnr_regulator_interface/internal/cnr_regulator_base.h>

namespace cnr
{
namespace control
{

void get_resource_names(ros::NodeHandle* nh, std::vector<std::string>& names)
{
  std::vector<std::string> alternative_keys =
    {"controlled_resources", "controlled_resource", "controlled_joints", "controlled_joint", "joint_names", "joint_name"};

  names.clear();
  for(auto const & key : alternative_keys)
  {
    if(!nh->getParam(key, names))
    {
      std::string joint_name;
      if(nh->getParam(key, joint_name))
      {
        names.push_back(joint_name);
      }
    }
  }
  return;
}

bool BaseRegulator::initialize(ros::NodeHandle& /*root_nh*/,
                               ros::NodeHandle& controller_nh,
                               BaseRegulatorParamsPtr params)
{
  if(!params)
  {
    ROS_ERROR("BaseRegulator::initialize Null pointer to options! Abort.");
    return false;
  }
  if(!params->logger)
  {
    ROS_ERROR("BaseRegulator::initialize Null pointer to logger! Abort.");
    return false;
  }
  p_ = params;
  CNR_TRACE_START(params->logger);
  
  regulator_time_ = ros::Duration(0);
  
  get_resource_names(&controller_nh, controlled_resources_);
  if((controlled_resources_.size()==1)&&(controlled_resources_.front() == "all"))
  {
    controlled_resources_.clear();
    controlled_resources_ = params->resources_names;
  }

  CNR_RETURN_TRUE(p_->logger);
}

bool BaseRegulator::starting(BaseRegulatorStateConstPtr state0, const ros::Time& /*time*/)
{
  if(!state0)
  {
    ROS_ERROR("BaseRegulator::initialize Null pointer to null kinematic! Abort.");
    return false;
  }
  setRegulatorTime(ros::Duration(0.0));
  x0_ = state0;
  return true;
}

bool BaseRegulator::update( BaseRegulatorReferenceConstPtr   r,
                            BaseRegulatorFeedbackConstPtr    y,
                            BaseRegulatorControlCommandPtr   u)
{
  r_ = r;
  y_ = y;
  u_ = u;
  return true;
}

bool BaseRegulator::update( BaseRegulatorReferenceConstPtr   r,
                            BaseRegulatorControlCommandPtr   u)
{
  r_ = r;
  u_ = u;
  return true;
}

bool BaseRegulator::update( BaseRegulatorFeedbackConstPtr    y,
                            BaseRegulatorControlCommandPtr   u)
{
  y_ = y;
  u_ = u;
  return true;
}

bool BaseRegulator::update( BaseRegulatorControlCommandPtr   u)
{
  u_ = u;
  return true;
}

}  // namespace control
}  // namespace cnr
