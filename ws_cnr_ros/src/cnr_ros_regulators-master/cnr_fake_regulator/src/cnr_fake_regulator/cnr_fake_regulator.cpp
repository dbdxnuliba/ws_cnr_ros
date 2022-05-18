#include <cnr_fake_regulator/cnr_fake_regulator.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_regulator_interface/cnr_regulator_references.h>
#include <cnr_regulator_interface/cnr_regulator_control_commands.h>

#include <pluginlib/class_list_macros.h> 

PLUGINLIB_EXPORT_CLASS(cnr::control::FakeRegulator , cnr::control::BaseRegulator)
