// This header file defines the MyController class, which is a controller for a Franka robot.
// It implements the control logic for the robot's joints and end-effector based on the desired
// behavior specified by the user or through dynamic reconfiguration.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

// MyController class inherits from MultiInterfaceController to control the Franka robot.
class MyController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  // Function to initialize the controller.
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  // Function called when the controller starts.
  void starting(const ros::Time&) override;
  // Function called periodically to update the controller.
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation function to limit torque rate.
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // Handles for accessing robot state and model information.
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  // Vector of handles for controlling the robot's joints.
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Parameters for filtering and controlling the nullspace stiffness.
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{0.2};

  // Matrices for controlling Cartesian stiffness and damping.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  // Desired nullspace equilibrium configuration and target position and orientation.
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Dynamic reconfigure server for handling compliance parameters.
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Subscriber for receiving equilibrium pose commands.
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_example_controllers
