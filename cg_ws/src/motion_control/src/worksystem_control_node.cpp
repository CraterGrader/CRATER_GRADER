#include "motion_control/worksystem_control_node.hpp"

namespace cg {
namespace motion_control {

WorksystemControlNode::WorksystemControlNode() : Node("worksystem_control_node") {

  // Initialize services
  update_trajectory_server_ = this->create_service<cg_msgs::srv::SendTrajectory>(
      "update_trajectory_server",
      std::bind(&WorksystemControlNode::updateTrajectory, this, std::placeholders::_1, std::placeholders::_2));

  // Load parameters
  this->declare_parameter<double>("longitudinal_velocity_kp", 1.0);
  this->get_parameter("longitudinal_velocity_kp", longitudinal_velocity_kp_);
  this->declare_parameter<double>("longitudinal_velocity_ki", 0.0);
  this->get_parameter("longitudinal_velocity_ki", longitudinal_velocity_ki_);
  this->declare_parameter<double>("longitudinal_velocity_kd", 0.0);
  this->get_parameter("longitudinal_velocity_kd", longitudinal_velocity_kd_);
  this->declare_parameter<double>("lateral_stanley_gain", 1.0);
  this->get_parameter("lateral_stanley_gain", lateral_stanley_gain_);
}

void WorksystemControlNode::updateTrajectory(cg_msgs::srv::SendTrajectory::Request::SharedPtr req, cg_msgs::srv::SendTrajectory::Response::SharedPtr res)
{
  current_trajectory_ = req->trajectory; // Update current trajectory
  res->updated_trajectory = true; // Set response confirmation
}

}  // namespace motion_control
}  // namespace cg
