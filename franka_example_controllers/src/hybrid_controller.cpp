// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hybrid_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <algorithm>

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

  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // Subscribe to the "equilibrium_pose" topic to receive pose commands
  sub_equilibrium_pose_ = node_handle.subscribe(
    "equilibrium_pose", 20, &HybridController::equilibriumPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void HybridController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ROS_INFO("NEW MESSAGE RECEIVED");
  desired_pose_msg = *msg;
}

Eigen::Vector3d saturateError(const Eigen::Vector3d& error, double max_error) {
  Eigen::Vector3d saturated_error;
  for (int i = 0; i < 3; ++i) {
    if (error[i] > max_error) {
      saturated_error[i] = max_error;
    } else if (error[i] < -max_error) {
      saturated_error[i] = -max_error;
    } else {
      saturated_error[i] = error[i];
    }
  }
  return saturated_error;
}

void HybridController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

void HybridController::update(const ros::Time& /* time */, const ros::Duration& /* period */) {
  std::array<double, 16> new_pose = initial_pose_;

  const double position_gain = 1;
  const double max_position_error = 0.00001;

  std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  Eigen::Vector3d current_position(new_pose[12], new_pose[13], new_pose[14]);
  Eigen::Vector3d desired_position(desired_pose_msg.pose.position.x, 
                                   desired_pose_msg.pose.position.y, 
                                   desired_pose_msg.pose.position.z);

  Eigen::Vector3d position_error = desired_position - current_position;
  position_error = position_gain * position_error;
  position_error = saturateError(position_error, max_position_error);

  // ROS_INFO("position_error: %f", position_error.x());
  new_pose[12] = current_position.x() + position_error.x();
  new_pose[13] = current_position.y() + position_error.y();
  new_pose[14] = current_position.z() + position_error.z();

  cartesian_pose_handle_->setCommand(new_pose);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HybridController, controller_interface::ControllerBase)
