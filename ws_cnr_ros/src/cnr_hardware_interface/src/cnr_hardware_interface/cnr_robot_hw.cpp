/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_hardware_interface/internal/vector_to_string.h>
#include <cnr_hardware_interface/cnr_robot_hw.h>

#define GENERATE_ENUM_STRINGS  // Start string generation
#include <cnr_hardware_interface/cnr_robot_hw_status.h>
#undef GENERATE_ENUM_STRINGS   // Stop string generation

namespace cnr_hardware_interface
{

inline
void get_resource_names(ros::NodeHandle& nh, std::vector<std::string>& names)
{
  std::vector<std::string> alternative_keys =
    { "controlled_resources", "controlled_resource",
      "controlled_joints", "controlled_joint",
      "joint_names", "joint_name",
      "joints", "joint",
      "joint_resource/joint_names", "joint_resource/joint_name"
      "joint_resource/controlled_joints", "joint_resource/controlled_joint"
      "joint_resource/controlled_resources", "joint_resource/controlled_resource"
      };

  names.clear();
  for(auto const & key : alternative_keys)
  {
    bool ok = false;
    std::string what;
    try
    {
      ok = rosparam_utilities::get(nh.getNamespace() + "/" + key, names, what);
    }
    catch(const std::exception& e)
    {
      std::cerr <<"Exception in getting '"<< nh.getNamespace() << "/" << key << "': " << e.what() << '\n';
      ok = false;
    }
    
    if(!ok)
    {
      try 
      {
        std::string joint_name;
        if(rosparam_utilities::get(nh.getNamespace() + "/" + key, joint_name, what))
        {
          names.push_back(joint_name);
        }
      }
      catch(const std::exception& e)
      {
        std::cerr <<"Exception in getting '"<< nh.getNamespace() << "/" << key << "': " << e.what() << '\n';
        ok = false;
      }
    }

    if(ok)
    {
      break;
    }
  }
  return;
}
  
RobotHW::RobotHW()
  : m_set_status_param(nullptr), m_is_first_read(true), m_state(cnr_hardware_interface::CREATED),
    m_state_prev(cnr_hardware_interface::CREATED), m_shutted_down(false)
{
  setState(cnr_hardware_interface::CREATED);
}

RobotHW::~RobotHW()
{
  CNR_TRACE_START(m_logger);
  if(!m_shutted_down)
  {
    if(!shutdown())
    {
      setState(cnr_hardware_interface::ERROR);
    }
    else
    {
      setState(cnr_hardware_interface::SHUTDOWN);
    }
  }
  CNR_TRACE(m_logger, "[  DONE] ");
}

bool RobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robothw_nh)
{
  std::string n = "HW" + robothw_nh.getNamespace();
  std::replace(n.begin(), n.end(), '/', '_');

  std::string what;
  if (!m_logger.init(n, robothw_nh.getNamespace(), false, false, &what))
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << "Error in creating the logger!" << std::endl;
    std::cerr <<  "what:" << what << std::endl;
    return false;
  }
  
  CNR_TRACE_START(m_logger);
  if(enterInit(root_nh, robothw_nh) && doInit() && exitInit())
  {
    CNR_RETURN_TRUE(m_logger, "RobotHW '" + m_robot_name + "' Initialization OK");
  }
  
  CNR_RETURN_FALSE(m_logger,  "RobotHW '" + m_robot_name + "' Initialization Failed");
}

void RobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);

  if(m_is_first_read)
  {
    setState(cnr_hardware_interface::RUNNING);
    m_is_first_read = false;
    if(m_set_status_param)
    {
      m_set_status_param("first_configuration");
    }
  }

  m_robot_hw_queue.callAvailable();

  if(!doRead(time, period))
  {
    setState(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK_THROTTLE(m_logger, void(), 5.0);
  }

  CNR_RETURN_OK_THROTTLE_DEFAULT(m_logger, void());
}

void RobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(m_logger);
  if(!doWrite(time, period))
  {
    setState(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK_THROTTLE(m_logger, void(), 5.0, "Error in writing...");  
  }
  
  if(m_state_prev != getState())
  {
     if(m_set_status_param)
     {
       m_set_status_param("last_valid_configuration");
     }
     setState(getState()); // re-setting the state, I also change the m_state_prev
  }
  CNR_RETURN_OK_THROTTLE_DEFAULT(m_logger, void());
}

// THE FUNCTION IS CALLED JUST BEFORE PREPARE SWITCH.
// THIS FUNCITON DOES NOT CHANGE THE STATE IF EVERITHING IS OK. 
// THE STATE IS CHAGED AT THE END OF PREPARE SWITCH
// THE STATE IS CHANGED ON ERROR JUST IN CASE OF ISSUES
bool RobotHW::checkForConflict(const std::list< hardware_interface::ControllerInfo >& info) const
{
  CNR_TRACE_START(m_logger);
  if(::hardware_interface::RobotHW::checkForConflict(info))
  {
    setState(cnr_hardware_interface::ERROR);
    std::string result = cnr_logger::RED() + "[ERROR] Base Check failed" + cnr_logger::RESET();
    CNR_TRACE(m_logger, result << __FUNCTION__);
    return true;
  }

  // Each controller can use more than a hardware_interface for a single joint (e.g.: position, velocity, effort)
  // One controller can control more than one joint. A joint can be used only by a controller.
  std::vector<bool> global_resource_used(m_resource_names.size());
  std::fill(global_resource_used.begin(), global_resource_used.end(), false);

  for (hardware_interface::ControllerInfo controller : info)
  {
    std::vector<bool> single_controller_resource_used(m_resource_names.size());
    std::fill(single_controller_resource_used.begin(), single_controller_resource_used.end(), false);

    for (hardware_interface::InterfaceResources res : controller.claimed_resources)
    {
      for (std::string name : res.resources)
      {
        for (unsigned int iJ = 0; iJ < m_resource_names.size(); iJ++)
        {
          if(!name.compare(m_resource_names.at(iJ)))
          {
            if(global_resource_used.at(iJ))   // if already used by another
            {
              CNR_FATAL(m_logger, "Joint " + name + "%s is already used by another controller" ); 
              setState(cnr_hardware_interface::CTRL_ERROR);
              std::string result = cnr_logger::RED() + "[ERROR] " + cnr_logger::RESET();
              CNR_TRACE(m_logger, result << __FUNCTION__);
              return true;
            }
            else
            {
              single_controller_resource_used.at(iJ) = true;
            }
          }
        }
      }
    }
    for (unsigned int iJ = 0; iJ < m_resource_names.size() ; iJ++)
    {
      global_resource_used.at(iJ) = global_resource_used.at(iJ) || single_controller_resource_used.at(iJ);
    }
  }

  if(doCheckForConflict(info))
  {
    setState(cnr_hardware_interface::ERROR);
    std::string result = cnr_logger::RED() + "[ERROR] " + cnr_logger::RESET();
    CNR_TRACE(m_logger, result << __FUNCTION__);
    return true;
  }

  std::string result = cnr_logger::GREEN() + "[  DONE] " + cnr_logger::RESET();
  CNR_TRACE(m_logger, result << __FUNCTION__);
  return false;
}

bool RobotHW::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                            const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  CNR_TRACE_START(m_logger, "************************ PREPARE SWITCH OF CONTROLLERS (NO RT) ****************************************");
  CNR_DEBUG(m_logger, "RobotHW '" << m_robothw_nh.getNamespace()
                        << "' Status " <<  cnr_hardware_interface::to_string(getState()));
  if(getState()==cnr_hardware_interface::ERROR
  || getState()==cnr_hardware_interface::CTRL_ERROR
  || getState()==cnr_hardware_interface::SRV_ERROR)
  {
    CNR_RETURN_FALSE(m_logger, "The controller switch is not possible, since the RobotHw is in ERROR state.");
  }

  //====== CHECK COHERENCE WITH ACTUAL STORED STATE
  std::stringstream report;
  for (const hardware_interface::ControllerInfo& ctrl : stop_list)
  {
    std::list<hardware_interface::ControllerInfo>::iterator stopped_controller =
      std::find_if(m_active_controllers.begin(), m_active_controllers.end(),
                   [&ctrl](hardware_interface::ControllerInfo & it) { return (it.name == ctrl.name); });  //NOLINT

    if(m_active_controllers.end() == stopped_controller)
    {
      addDiagnosticsMessage("ERROR", "controller '" + ctrl.name + "' is not active, so I cannot stop it",
                              { {"Transition", "switching"} }, &report);  //NOLINT
      setState(cnr_hardware_interface::CTRL_ERROR);
      CNR_RETURN_FALSE(m_logger, report.str());
    }
    m_active_controllers.erase(stopped_controller);
  }
  for (const hardware_interface::ControllerInfo& ctrl : start_list)
  {
    std::list<hardware_interface::ControllerInfo>::iterator to_start_controller =
      std::find_if(m_active_controllers.begin(), m_active_controllers.end(),
                   [&ctrl](hardware_interface::ControllerInfo & it) { return (it.name == ctrl.name); });  //NOLINT
    if(m_active_controllers.end() != to_start_controller)
    {
      CNR_WARN(m_logger, "controller " << ctrl.name << "is already active, so I cannot start it");
    }
    else
    {
      m_active_controllers.push_back(ctrl);
    }
  }
  //====== CHECK COHERENCE WITH ACTUAL STORED STATE


  if(!doPrepareSwitch(start_list, stop_list))
  {
    CNR_RETURN_FALSE(m_logger);
  }

  setState(cnr_hardware_interface::READY_TO_SWITCH);

  m_is_first_read = false;
  CNR_RETURN_TRUE(m_logger, "************************ PREPARE SWITCH OF CONTROLLERS (NO RT) - END **********************************");
}


void RobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                       const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  CNR_TRACE_START(m_logger,"************** DO SWITCH OF CONTROLLERS (IN RT UPDATE) *****************************");
  CNR_DEBUG(m_logger, "RobotHW '" << m_robothw_nh.getNamespace()
                        << "' Status " <<  cnr_hardware_interface::to_string(getState()));
  if(getState()==cnr_hardware_interface::ERROR
  || getState()==cnr_hardware_interface::CTRL_ERROR
  || getState()==cnr_hardware_interface::SRV_ERROR)
  {
    CNR_ERROR(m_logger, "The controller switch is not possible, since the RobotHw is in ERROR state.");
    CNR_RETURN_NOTOK(m_logger, void());
  }

  setState(cnr_hardware_interface::DOING_SWITCH);
  if(!doDoSwitch(start_list, stop_list))
  {
    setState(cnr_hardware_interface::ERROR);
    CNR_RETURN_NOTOK(m_logger, void());
  }
  setState(cnr_hardware_interface::SWITCH_DONE);

  CNR_TRACE(m_logger, "************** DO SWITCH OF CONTROLLERS (IN RT UPDATE)- END ************************");
  CNR_RETURN_OK(m_logger, void());
}

hardware_interface::RobotHW::SwitchState RobotHW::switchResult() const
{
    return getState() == cnr_hardware_interface::ERROR ? hardware_interface::RobotHW::SwitchState::ERROR : 
           getState() == cnr_hardware_interface::DOING_SWITCH ? hardware_interface::RobotHW::SwitchState::ONGOING : 
           getState() == cnr_hardware_interface::SWITCH_DONE ? hardware_interface::RobotHW::SwitchState::DONE : 
           getState() == cnr_hardware_interface::RUNNING ? hardware_interface::RobotHW::SwitchState::DONE : 
           hardware_interface::RobotHW::SwitchState::ERROR;
}

// TODO manage each controller
hardware_interface::RobotHW::SwitchState RobotHW::switchResult(const hardware_interface::ControllerInfo& controller) const
{
    return getState() == cnr_hardware_interface::ERROR ? hardware_interface::RobotHW::SwitchState::ERROR : 
           getState() == cnr_hardware_interface::DOING_SWITCH ? hardware_interface::RobotHW::SwitchState::ONGOING : 
           getState() == cnr_hardware_interface::RUNNING ? hardware_interface::RobotHW::SwitchState::DONE : 
           hardware_interface::RobotHW::SwitchState::ERROR;
}



bool RobotHW::shutdown()
{
  CNR_TRACE_START(m_logger, ">>>> RobotHW Shutdown (" + m_robot_name + ")");
  if(!doShutdown())
  {
    setState(cnr_hardware_interface::ERROR);
    CNR_RETURN_FALSE(m_logger, "<<<< Robot Shutdown Failure (" + m_robot_name + ")");
  }

  if(m_set_status_param)
  {
    m_set_status_param("shutdown_configuration");
  }
  m_shutted_down =  true;
  setState(cnr_hardware_interface::SHUTDOWN);
  CNR_RETURN_TRUE(m_logger, "<<<< Robot Shutdown (" + m_robot_name + ")");
}



bool RobotHW::enterInit(ros::NodeHandle& root_nh, ros::NodeHandle& robothw_nh)
{
  CNR_TRACE_START(m_logger);
  m_robothw_nh  = robothw_nh;
  m_robothw_nh.setCallbackQueue(&m_robot_hw_queue);
  m_root_nh     = root_nh;
  m_stop_thread = false;
  m_robot_name  = extractRobotName(m_robothw_nh.getNamespace());
  m_set_param   = m_robothw_nh.advertiseService("writeParams", &cnr_hardware_interface::RobotHW::setParamServer, this);
  m_get_param   = m_robothw_nh.advertiseService("readParams", &cnr_hardware_interface::RobotHW::getParamServer, this);

  m_robot_hw_queue.callAvailable();

  realtime_utilities::DiagnosticsInterface::init(m_robot_name, "RobotHW", m_robot_name );
  
  get_resource_names(m_robothw_nh,m_resource_names);
  if(m_resource_names.size()==0)
  {
    setState(cnr_hardware_interface::ERROR);
    CNR_RETURN_FALSE(m_logger, "Neither '" +  m_robothw_nh.getNamespace() + "/controlled_joint(s)' nor '"
                        +  m_robothw_nh.getNamespace() + "/controlled_resources(s)' are specified. Abort" );
  }
  CNR_DEBUG(m_logger, "Resources: " << cnr_hardware_interface::to_string(m_resource_names));

  std::string what;
  if(!rosparam_utilities::get(m_robothw_nh.getNamespace() + "/sampling_period", m_sampling_period, what))
  {
    m_sampling_period = 1e-3;
    CNR_WARN(m_logger, "Sampling period not found");
  }
  CNR_RETURN_TRUE(m_logger);
}

bool RobotHW::exitInit()
{
  CNR_TRACE_START(m_logger);
  bool ret = (m_resource_names.size() > 0);

  if(!ret)
  {
    CNR_FATAL(m_logger, "Reources names not set! Remeber to assign them in the doInit function. ");
  }

  setState(ret ? cnr_hardware_interface::INITIALIZED : cnr_hardware_interface::ERROR);
  CNR_RETURN_BOOL(m_logger, ret);
}




bool RobotHW::getParamServer(configuration_msgs::GetConfigRequest& req, configuration_msgs::GetConfigResponse& res)
{
  return true;
}

bool RobotHW::setParamServer(configuration_msgs::SetConfigRequest& req, configuration_msgs::SetConfigResponse& res)
{
  return true;
}

bool RobotHW::setState(const cnr_hardware_interface::StatusHw& status) const
{
  // if((m_state_history.size() == 0) || (m_state_history.back() != cnr_hardware_interface::to_string(getState())))
  // {
  //   m_state_history.push_back(cnr_hardware_interface::to_string(getState()));
  // }
  m_state_prev = m_state;
  m_state = status;
  return true;
}


}  // namespace cnr_hardware_interface
