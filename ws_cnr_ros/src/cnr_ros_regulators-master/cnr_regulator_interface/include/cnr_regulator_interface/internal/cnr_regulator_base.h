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

#ifndef CNR_REGULATOR_INTERFACE__CNR_REGULATOR_BASE__H
#define CNR_REGULATOR_INTERFACE__CNR_REGULATOR_BASE__H

#include <ros/node_handle.h>
#include <Eigen/Dense>
#include <memory>
#include <cnr_logger/cnr_logger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <rosdyn_chain_state/chain_state.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>
#include <cnr_regulator_interface/cnr_regulator_params.h>
#include <cnr_regulator_interface/cnr_regulator_state.h>
#include <cnr_regulator_interface/cnr_regulator_references.h>
#include <cnr_regulator_interface/cnr_regulator_control_commands.h>
#include <cnr_regulator_interface/cnr_regulator_feedback.h>

namespace cnr
{
namespace control
{

/**
 * @brief The BaseRegulator class
 */
class BaseRegulator
{
public:
  BaseRegulator() = default;
  virtual ~BaseRegulator() = default;
  BaseRegulator(const BaseRegulator&) = delete;
  BaseRegulator& operator=(const BaseRegulator&) = delete;
  BaseRegulator(BaseRegulator&&) = delete;
  BaseRegulator& operator=(BaseRegulator&&) = delete;

  virtual bool initialize(ros::NodeHandle&  root_nh, 
                          ros::NodeHandle& controller_nh, 
                          BaseRegulatorParamsPtr opts);

  virtual bool starting(BaseRegulatorStateConstPtr state0,
                        const ros::Time& /*time*/);

  virtual bool update(BaseRegulatorReferenceConstPtr /*r*/,
                      BaseRegulatorFeedbackConstPtr  /*y*/,
                      BaseRegulatorControlCommandPtr /*u*/);

  virtual bool update(BaseRegulatorFeedbackConstPtr  /*y*/,
                      BaseRegulatorControlCommandPtr /*u*/);

  virtual bool update(BaseRegulatorReferenceConstPtr /*r*/,
                      BaseRegulatorControlCommandPtr /*u*/);

  virtual bool update(BaseRegulatorControlCommandPtr /*u*/);

  virtual bool stopping(const ros::Time& /*time*/) {return true;}

  void setRegulatorTime(const ros::Duration& time) { regulator_time_ = time;}
  const ros::Duration& getRegulatorTime() const { return regulator_time_; }

  InterpolatorBasePtr      interpolator() { return p_->interpolator; };
  InterpolatorBaseConstPtr interpolator() const { return p_->interpolator; };
  
  cnr_logger::TraceLoggerPtr     logger()        { return p_->logger;    }
  const size_t&                  dim   () const  { return p_->dim;       }
  const ros::Duration&           period() const  { return p_->period;    }

protected:
  ros::Duration                 regulator_time_;
  std::vector<std::string>      controlled_resources_;
  
  BaseRegulatorParamsPtr         p_;  
  BaseRegulatorStateConstPtr     x0_;
  BaseRegulatorStatePtr          x_;
  BaseRegulatorReferenceConstPtr r_;
  BaseRegulatorControlCommandPtr u_;
  BaseRegulatorFeedbackConstPtr  y_;
};

typedef std::shared_ptr<BaseRegulator> BaseRegulatorPtr;
typedef std::shared_ptr<BaseRegulator const> BaseRegulatorConstPtr;
}
}

#endif  // CNR_REGULATOR_INTERFACE__CNR_REGULATOR_BASE__H
