// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool MyController::init(hardware_interface::RobotHW* robot_hw,
                        ros::NodeHandle& node_handle) {
  // Vectors for stiffness and damping parameters
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // Subscribe to the "equilibrium_pose" topic to receive pose commands
  sub_equilibrium_pose_ = node_handle.subscribe(
    "equilibrium_pose", 20, &MyController::equilibriumPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  // Retrieve the arm_id parameter from the node handle
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("MyController: Could not read parameter arm_id");
    return false;
  }

  // Retrieve the joint names parameter and ensure it contains exactly 7 names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "MyController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Get the Franka model interface from the hardware
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("MyController: Error getting model interface from hardware");
    return false;
  }

  // Initialize the model handle
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MyController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // Get the Franka state interface from the hardware
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("MyController: Error getting state interface from hardware");
    return false;
  }

  // Initialize the state handle
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MyController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // Get the effort joint interface from the hardware
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("MyController: Error getting effort joint interface from hardware");
    return false;
  }

  // Initialize the joint handles for all 7 joints
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("MyController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Set up the dynamic reconfigure server for compliance parameters
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&MyController::complianceParamCallback, this, _1, _2));

  // Initialize desired position and orientation to zero (identity quaternion)
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // Initialize stiffness and damping matrices to zero
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // Return true to indicate successful initialization
  return true;
}

void MyController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void MyController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control
  tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));

  // nullspace PD control
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                    (2.0 * sqrt(nullspace_stiffness_)) * dq);

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;

  ROS_INFO_STREAM("cartesian stiffness: " << cartesian_stiffness_);
  ROS_INFO_STREAM("cartesian damping: " << cartesian_damping_);
  ROS_INFO_STREAM("tau_task: " << tau_task.transpose());
  ROS_INFO_STREAM("tau_nullspace: " << tau_nullspace.transpose());
  ROS_INFO_STREAM("coriolis: " << coriolis.transpose());
  ROS_INFO_STREAM("tau_d before saturation: " << tau_d.transpose());

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  ROS_INFO_STREAM("tau_d after saturation: " << tau_d.transpose());

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> MyController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  // Initialize the saturated torque vector
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};

  // Iterate through each torque component
  for (size_t i = 0; i < 7; i++) {
    // Calculate the difference between the calculated torque and the joint torque
    double difference = tau_d_calculated[i] - tau_J_d[i];
    // Saturate the torque difference within a specified range
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }

  // Return the saturated torque vector
  return tau_d_saturated;
}

void MyController::complianceParamCallback(franka_example_controllers::compliance_paramConfig& config, uint32_t /*level*/) {
  // Set the target Cartesian stiffness matrix
  cartesian_stiffness_target_.setIdentity();
  // Set translational stiffness using the provided config
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  // Set rotational stiffness using the provided config
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();

  // Set the target Cartesian damping matrix
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  // Set translational damping using the provided config
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  // Set rotational damping using the provided config
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();

  // Set the target nullspace stiffness
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void MyController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // Lock the mutex to ensure thread safety while updating the target position and orientation.
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);

  // Update the target position of the end effector to the values received in the message.
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  // Store the current target orientation before updating it.
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);

  // Update the target orientation of the end effector to the values received in the message.
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

  // Ensure continuity in the quaternion representation by checking if the dot product
  // between the old and new orientation is negative. If it is, negate the new orientation
  // coefficients to keep the orientation in the same hemisphere.
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyController,
                       controller_interface::ControllerBase)
