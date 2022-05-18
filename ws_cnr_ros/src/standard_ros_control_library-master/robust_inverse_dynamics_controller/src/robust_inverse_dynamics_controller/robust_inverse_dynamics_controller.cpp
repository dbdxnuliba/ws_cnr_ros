#include <robust_inverse_dynamics_controller/robust_inverse_dynamics_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_control::RobustInverseDynamicsControl, controller_interface::ControllerBase);


namespace robot_control
{

bool RobustInverseDynamicsControl::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

  m_hw=hw;
  m_controller_nh=controller_nh;
  m_root_nh=root_nh;

  // Loading chain
  if (!m_controller_nh.getParam("base_frame",m_base_frame))
  {
    ROS_ERROR("%s/base_link not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("tool_frame",m_tool_frame))
  {
    ROS_ERROR("tool_link not defined");
    return false;
  }
  urdf::Model urdf_model;
  if (!urdf_model.initParam("/robot_description"))
  {
    ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
    return false;
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  m_chain = rosdyn::createChain(urdf_model,m_base_frame,m_tool_frame,gravity);
  m_joint_names=m_chain->getMoveableJointNames();

  // check joint names
  bool flag=false;
  for (std::string joint_name: m_joint_names)
  {
    for (unsigned idx=0;idx<m_hw->getNames().size();idx++)
    {
      if (!m_hw->getNames().at(idx).compare(joint_name))
      {
        m_joint_handles.push_back(m_hw->getHandle(joint_name));
        ROS_DEBUG("ADD %s handle",joint_name.c_str());
        flag=true;
        break;
      }
    }
    if (!flag)
    {
      ROS_FATAL("Joint %s is not managed",joint_name.c_str());
      return false;
    }
  }

  // set target topic name
  std::string setpoint_topic_name;
  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_js_rec.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh,setpoint_topic_name, 1,boost::bind(&RobustInverseDynamicsControl::setTargetCallback,this,_1)));

  ROS_DEBUG("Controller '%s' controls the following joint:",m_controller_nh.getNamespace().c_str());
  for (std::string& name: m_joint_names)
    ROS_DEBUG("- %s",name.c_str());

  m_nax=m_joint_names.size();

  // set natural frequency and damping
  if (not m_ctrl.init(controller_nh,m_chain))
  {
    ROS_ERROR("Unable to initialize controller");
    return false;
  }

  m_velocity.resize(m_nax);
  m_position.resize(m_nax);
  m_target_position.resize(m_nax);
  m_target_velocity.resize(m_nax);
  m_target_effort.resize(m_nax);

  m_velocity.setZero();
  m_position.setZero();
  m_target_position.setZero();
  m_target_velocity.setZero();
  m_target_effort.setZero();
  return true;
}

void RobustInverseDynamicsControl::starting(const ros::Time& /*time*/)
{
  m_configured = false;

  for (unsigned int iax=0;iax<m_nax;iax++)
  {
    m_position(iax) = m_joint_handles.at(iax).getPosition();
    m_velocity(iax) = m_joint_handles.at(iax).getVelocity();
  }
  m_target_position=m_position;
  m_queue.callAvailable();
  m_ctrl.starting(m_position,m_velocity);
}

void RobustInverseDynamicsControl::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller", m_controller_nh.getNamespace().c_str());
  m_ctrl.stopping();
  m_configured = false;
}

void RobustInverseDynamicsControl::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{

  m_queue.callAvailable();

  for (unsigned int iax=0;iax<m_nax;iax++)
  {
    m_position(iax) = m_joint_handles.at(iax).getPosition();
    m_velocity(iax) = m_joint_handles.at(iax).getVelocity();
  }

  m_ctrl.update(m_position,
                m_velocity,
                m_target_position,
                m_target_velocity,
                m_target_effort);

  for (unsigned int iax=0;iax<m_nax;iax++)
  {
    m_joint_handles.at(iax).setCommand(m_target_effort(iax));
  }
}


void RobustInverseDynamicsControl::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(m_joint_names,tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
    {
      ROS_ERROR_THROTTLE(1,"joints not found");
      m_configured=false;
      return;
    }
    if (!m_configured)
      ROS_INFO("First target message received");

    m_configured=true;
    for (unsigned int iAx=0;iAx<m_nax;iAx++)
    {
      m_target_position(iAx) = tmp_msg.position.at(iAx);
      m_target_velocity(iAx) = tmp_msg.velocity.at(iAx);
      m_target_effort(iAx)   = tmp_msg.effort.at(iAx);
    }
  }
  catch(...)
  {
    ROS_ERROR("something wrong in target callback");
    m_configured=false;
  }
}
}  //  namespace robot_control
