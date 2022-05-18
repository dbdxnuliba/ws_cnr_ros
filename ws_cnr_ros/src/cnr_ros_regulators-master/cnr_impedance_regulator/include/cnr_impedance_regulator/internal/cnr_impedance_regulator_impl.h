#pragma once // workaraound qtcreator clang tidy

#ifndef CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR_IMPL__H
#define CNR_IMPEDANCE_REGULATOR__CNR_IMPEDANCE_REGULATOR__H

#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

//!
inline bool ImpedanceRegulator::initialize(ros::NodeHandle&  root_nh,
                                    ros::NodeHandle&  controller_nh,
                                    cnr::control::BaseRegulatorParamsPtr opts)
{
  if(!this->BaseImpedanceRegulator::initialize(root_nh,controller_nh,opts))
  {
    return false;
  }
  try
  {
    eu::resize(m_Jinv,this->dim());
    eu::resize(m_damping,this->dim());
    eu::resize(m_damping_dafault,this->dim());
    eu::resize(m_k,this->dim());
    eu::resize(m_k_default,this->dim());
    eu::resize(m_k_new,this->dim());

    std::string what;
    std::vector<double> inertia, damping, stiffness, eff_deadband, damping_ratio;
    if(!rosparam_utilities::getParam(controller_nh, "inertia", inertia, what))
    {
      CNR_RETURN_FALSE(this->logger(),what);
    }
    if(inertia.size()!=this->dim())
    {
      CNR_RETURN_FALSE(this->logger(),"The dimension of the param 'inertia' mismatches with the expected dimension");
    }
    if(!rosparam_utilities::getParam(controller_nh, "stiffness", stiffness, what))
    {
      CNR_RETURN_FALSE(this->logger(),what);
    }
    if(stiffness.size()!=this->dim())
    {
      CNR_RETURN_FALSE(this->logger(),"The dimension of the param 'stiffness' mismatches with the expected dimension");
    }
    if(rosparam_utilities::getParam(controller_nh, "damping_ratio",damping_ratio, what))
    {
      if(damping_ratio.size()!=this->dim())
      {
        CNR_RETURN_FALSE(this->logger(),"The dimension of the param 'damping_ratio' mismatches with the expected dimension");
      }

      damping.resize(this->dim(),0);
      for(unsigned int iAx=0; iAx<this->dim(); iAx++)
      {
        if(stiffness.at(iAx)<=0)
        {
          ROS_ERROR("damping ratio can be specified only for positive stiffness values (stiffness of Joint is not positive)");
          return false;
        }
        damping.at(iAx)=2*damping_ratio.at(iAx)*std::sqrt(stiffness.at(iAx)*inertia.at(iAx));
      }
    }
    else if(!rosparam_utilities::getParam(controller_nh, "damping",damping, what))
    {
      CNR_RETURN_FALSE(this->logger(),what);
    }
    if(damping.size()!=this->dim())
    {
      CNR_RETURN_FALSE(this->logger(),"The dimension of the param 'damping' mismatches with the expected dimension");
    }


    for (unsigned int iAx=0;iAx<this->dim();iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        CNR_INFO(this->logger(), "inertia value of Joint "+ std::to_string(iAx) +"is not positive, disabling impedance control for this axis");
        eu::at(m_Jinv,iAx)=0.0;
      }
      else
      {
        eu::at(m_Jinv,iAx)=1.0/inertia.at(iAx);
      }
      if (damping.at(iAx)<=0)
      {
        CNR_INFO(this->logger(), "damping value of Joint "+std::to_string(iAx)+" is not positive, setting equalt to 10/inertia");
        eu::at(m_damping,iAx)         = 10.0 * eu::at(m_Jinv,iAx);
        eu::at(m_damping_dafault,iAx) = 10.0 * eu::at(m_Jinv,iAx);
      }
      else
      {
        eu::at(m_damping,iAx)         = damping.at(iAx);
        eu::at(m_damping_dafault,iAx) = damping.at(iAx);
      }
      if (stiffness.at(iAx)<0)
      {
        CNR_INFO(this->logger(), "maximum fitness value of Joint " + std::to_string(iAx) + " is negative, setting equal to 0");
        eu::at(m_k,iAx)=0.0;
        eu::at(m_k_default,iAx)=0.0;
      }
      else
      {
        eu::at(m_k,iAx)=stiffness.at(iAx);
        eu::at(m_k_default,iAx)=stiffness.at(iAx);
      }
    }
  }
  catch(const  std::exception& e)
  {
    CNR_ERROR(this->logger(), "EXCEPTION: " << e.what());
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

//!
inline bool ImpedanceRegulator::starting(cnr::control::BaseRegulatorStateConstPtr state0, const ros::Time& time)
{
  if(!this->BaseImpedanceRegulator::starting(state0, time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

//!
inline bool ImpedanceRegulator::update(cnr::control::BaseRegulatorReferenceConstPtr _r,
                                cnr::control::BaseRegulatorControlCommandPtr _u)
{
  if(!BaseImpedanceRegulator::update(_r, _u))
  {
    CNR_RETURN_FALSE(this->logger());
  }

  // refernece - alias
  MassSpringDamperModel& msd = this->x()->msdState(); //mass spring damper model

  msd.xdd = m_Jinv * ( m_k * (this->r()->q - msd.x)
          + m_damping * (this->r()->qd - msd.xd) + this->r()->effort);
  msd.x  += msd.xd  * this->period().toSec() + msd.xdd*std::pow(this->period().toSec(),2.0)*0.5;
  msd.xd += msd.xdd * this->period().toSec();

  this->regulator_time_  += this->period();

  this->u()->x = msd.x;
  this->u()->xd = msd.xd;
  this->u()->xdd = msd.xdd;
  this->u()->time_from_start = this->regulator_time_;

  return true;
}


}
}

#endif
