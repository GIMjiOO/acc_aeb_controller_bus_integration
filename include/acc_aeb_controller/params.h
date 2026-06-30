#pragma once
//==============================================================================
//  params.h  —  ROS parameter loading and contract-based validation.
//  This is the only place (besides the node) that touches roscpp, keeping the
//  Params struct itself (types.h) free of ROS dependencies.
//==============================================================================
#include <ros/ros.h>
#include "acc_aeb_controller/types.h"

namespace acc_aeb {

// Reads every parameter from the private namespace, falling back to defaults::.
[[nodiscard]] Params loadParams(ros::NodeHandle& nh);

// Contract checks. On violation: logs each failure with ROS_FATAL and calls
// ros::shutdown() (mirrors v15.0 behaviour).
void validateParams(const Params& p);

} // namespace acc_aeb
