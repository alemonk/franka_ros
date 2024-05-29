// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hybrid_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool HybridController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "HybridController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("HybridController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "HybridController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("HybridController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "HybridController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "HybridController: Exception getting state handle: " << e.what());
    return false;
  }

  // Initialize desired position and orientation to zero (identity quaternion)
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // Subscribe to the "equilibrium_pose" topic to receive pose commands
  sub_equilibrium_pose_ = node_handle.subscribe(
    "equilibrium_pose", 20, &HybridController::equilibriumPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void HybridController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // ROS_INFO_STREAM("Received pose: " << msg->pose);
  // Lock the mutex to ensure thread safety while updating the target position and orientation.
  std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);

  position_d_target_ << msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z;

  orientation_d_target_.coeffs() << msg->pose.orientation.x, 
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z, 
                                    msg->pose.orientation.w;

  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }

  desired_pose_msg = *msg;
}

void HybridController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void HybridController::update(const ros::Time& /* time */, const ros::Duration& period) {
  elapsed_time_ += period;

  // Get the latest target position and orientation from the message
  std::array<double, 16> new_pose = initial_pose_;

  std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  float err_x = new_pose[12] - desired_pose_msg.pose.position.x;
  float err_y = new_pose[13] - desired_pose_msg.pose.position.y;
  float err_z = new_pose[14] - desired_pose_msg.pose.position.z;

  new_pose[12] -= err_x / 10000.0;
  new_pose[13] -= err_y / 10000.0;
  new_pose[14] -= err_z / 10000.0;

  cartesian_pose_handle_->setCommand(new_pose);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HybridController,
                       controller_interface::ControllerBase)
