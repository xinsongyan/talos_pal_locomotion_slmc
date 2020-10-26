/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#pragma once

#include <ariles/ariles_all.h>

#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/com_stabilizer_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/cop_box_constraint_6d_dynamic.h>

#include <wbc_tasks/constraint_dynamic_task.h>
#include <wbc_tasks/go_to_dynamic_task.h>
#include <wbc_tasks/torque_damping_dynamic_task.h>
#include <wbc_tasks/physics_constraint_dynamic_task.h>
#include <wbc_tasks/com_dynamic_task.h>
#include <wbc_tasks/reference_dynamic_posture.h>
#include <wbc_tasks/torque_limit_dynamic_task.h>
#include <wbc_tasks/unilateral_forces_dynamic.h>
#include <wbc_tasks/force_regularization_dynamic.h>
#include <wbc_tasks/joint_pos_limit_dynamic_task.h>
#include <wbc_tasks/dynamic_task_joint_pos_vel_acc_limits.h>
#include <wbc_tasks/cop_box_constraint_6d_dynamic.h>
#include <wbc_tasks/cop_tracking_6d_dynamic.h>
#include <wbc_tasks/friction_constraint_task.h>
#include <wbc_tasks/momentum_regulation_dynamic_task.h>
#include <wbc_tasks/force_distribution_ds_dynamic_task.h>
#include <wbc_tasks/wrench_distribution_ds_dynamic_task.h>
#include <wbc_tasks/normal_torque_distribution_ds_dynamic_task.h>
#include <wbc_tasks/physics_tools.h>
#include <wbc_tasks/torque_regularization_dynamic.h>
#include <wbc_tasks/dynamic/go_relative_to_dynamic_task.h>

#include <pal_ros_utils/reference/pose/pose_pointer_reference.h>
#include <pal_ros_utils/reference/vector/vector_pointer_reference.h>

#include <pal_locomotion/wbc/wbc_base.h>
#include <pluginlib/class_loader.h>
#include <hqp_solvers/hqp_solver.h>
#include <pal_locomotion/leg_trajectory/swing_leg_trajectory_base.h>

#include <pal_utils/time_profiler.h>

#include <pal_ros_utils/reference/vector/vector_reference_minjerk_dynamic_reconfigure.h>
#include <pal_ros_utils/reference/vector/vector_reference_dynamic_reconfigure.h>

#include <pal_locomotion/wbc/wbc_parameters.h>
#include <pal_locomotion/wbc/wbc_gains.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace pal_locomotion
{
class WalkingWBCTalosBase : public BipedWbcBase
{
public:
  WalkingWBCTalosBase()
  {
  }

  virtual ~WalkingWBCTalosBase();

  virtual bool configure(ros::NodeHandle &nh, const ros::Time &time,
                         const ros::Duration &dt, BipedParameters *bp,
                         const Eigen::VectorXd &initialQ, const Eigen::VectorXd &initialQDot,
                         const std::vector<std::string> &footNames,
                         const bool &zeroComXYGain) override;

  virtual void initializeTasks(ros::NodeHandle &nh, BipedParameters *bp,
                               const ros::Time &time, const Eigen::VectorXd &initialQ);


  void updateTasks(const ros::Time &time, const std::vector<Side> stanceLegIDs,
                   const std::vector<Side> swingLegIDs, const double &weightDistribution,
                   const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot,
                   const Eigen::Vector3d &comPosition, const Eigen::Vector3d &comVelocity,
                   const Eigen::Vector3d &comAcceleration,
                   const Eigen::Quaterniond &baseOrientation,
                   const Eigen::Vector3d &baseAngularVelocity,
                   const Eigen::Vector3d &baseAngularAcceleration,
                   const Eigen::Quaterniond &torsoOrientation,
                   const Eigen::Vector3d &torsoAngularVelocity,
                   const Eigen::Vector3d &torsoAngularAcceleration,
                   const std::vector<Eigen::Vector3d> &stance_leg_positions,
                   const std::vector<eQuaternion> &stance_leg_orientations,
                   const std::vector<Eigen::Vector3d> &swingLegPositions,
                   const std::vector<Eigen::Vector3d> &swingLegLinearVelocitys,
                   const std::vector<Eigen::Vector3d> &swingLegLinearAcceleartions,
                   const std::vector<Eigen::Quaterniond> &swingLegOrientations,
                   const std::vector<Eigen::Vector3d> &swingLegAngularVelocitys,
                   const std::vector<Eigen::Vector3d> &swingLegAngularAcceleartions,
                   const Eigen::VectorXd &desiredUpperBodyJointPositions,
                   const Eigen::VectorXd &desiredUpperBodyJointVelocitys,
                   const Eigen::VectorXd &desiredUpperBodyJointAccelerations,
                   const Eigen::Vector3d &desiredAngularMomentum,
                   const Eigen::Vector3d &desiredAngularMomentumVelocity,
                   const std::vector<Eigen::Vector3d> &ee_position,
                   const std::vector<eQuaternion> &ee_orientation,
                   const std::vector<Eigen::Vector3d> &ee_linear_velocity,
                   const std::vector<Eigen::Vector3d> &ee_angular_velocity,
                   const std::vector<Eigen::Vector3d> &ee_linear_acceleration,
                   const std::vector<Eigen::Vector3d> &ee_angular_acceleration,
                   pal_wbc::task_container_vector &constraint_tasks,
                   pal_wbc::task_container_vector &objective_tasks);

  void processSolution(
      const std::vector<Side> stanceLegIDs, const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot,
      Eigen::VectorXd &Tau, Eigen::Vector3d &floatingBaseDesiredLinearAcceleration,
      Eigen::Vector3d &floatingBaseDesiredAngularAcceleration,
      Eigen::VectorXd &jointStateDesiredAcceleration,
      std::vector<std::pair<Eigen::Vector3d, pal_wbc::ContactDescription>> &computed_forces,
      std::vector<std::pair<Eigen::Vector3d, pal_wbc::ContactDescription>> &computed_torques);

  std::vector<std::string> getJointNames();


protected:
  //  ros::NodeHandle nh_;
  std::vector<std::string> foot_names_;
  std::vector<std::string> end_effector_names_;
  ros::Duration dt_;

  double zeroComXYGain_;  // Used when the COM/Planner is responsible for acting as a PD

  Eigen::VectorXd generalized_acceleration_;
  Eigen::VectorXd solution_;  // Only used for dynamics

  boost::shared_ptr<pluginlib::ClassLoader<HQPSolver>> hqp_solver_loader_;
  HQPSolverPtr hqp_solver_;

  // WBC ALLOCATIONS
  // Targets
  std::vector<pal_robot_tools::ReferenceAbstractPtr> stance_leg_targets_;
  std::vector<pal_robot_tools::ReferenceAbstractPtr> swing_leg_targets_;
  pal_robot_tools::PointerReferencePtr base_target_;
  pal_robot_tools::PointerReferencePtr torso_target_;
  std::vector<pal_robot_tools::ReferenceAbstractPtr> ee_targets_;

  // End-effector operational space targets
  std::vector<pal_robot_tools::ReferenceAbstractPtr> end_effector_targets_;

  // Dynamics
  pal_wbc::UnilateralForcesDynamicsPtr unilateral_forces_task_;
  pal_wbc::PhysicsDynamicConstraintMetaTaskPtr dynamic_task_;
  std::vector<pal_wbc::TaskAbstractPtr> position_constraint_xy_tasks_;
  std::vector<pal_wbc::TaskAbstractPtr> position_constraint_z_tasks_;
  std::vector<pal_wbc::TaskAbstractPtr> orientation_constraint_roll_pitch_tasks_;
  std::vector<pal_wbc::TaskAbstractPtr> orientation_constraint_yaw_tasks_;

  // Tasks
  pal_wbc::GoToXYCOMDynamicMetaTaskPtr com_xy_task_;
  pal_wbc::GoToZCOMDynamicMetaTaskPtr com_z_task_;
  pal_wbc::GoToOrientationDynamicMetaTaskPtr orientation_com_task_;
  pal_wbc::GoToOrientationDynamicMetaTaskPtr orientation_upper_torso_task_;

  pal_wbc::ForceDistributionDsDynamicTaskMetaTaskPtr ds_force_distribution_;
  // pal_wbc::NormalTorqueDistributionDsDynamicTaskPtr ds_normal_distribution_;

  pal_wbc::ForceRegularizationDynamicTaskPtr force_regularization_;
  pal_wbc::TorqueRegularizationDynamicTaskPtr torque_regularization_;
  //  pal_wbc::TorqueDampingDynamicTaskAllJointsMetaTaskPtr joint_torque_regularization_;

  pal_robot_tools::ReferenceAbstractPtr foot_relative_pose_target_;
  pal_wbc::GoToRelativeDynamicTaskPtr foot_relative_pose_xy_task_;
  pal_wbc::GoToRelativeDynamicTaskPtr foot_relative_pose_yaw_task_;

  std::vector<pal_wbc::GoToPositionDynamicMetaTaskPtr> swing_leg_position_xy_tasks_;
  std::vector<pal_wbc::GoToPositionDynamicMetaTaskPtr> swing_leg_position_z_tasks_;
  std::vector<pal_wbc::GoToOrientationDynamicMetaTaskPtr> swing_leg_orientation_roll_pitch_tasks_;
  std::vector<pal_wbc::GoToOrientationDynamicMetaTaskPtr> swing_leg_orientation_yaw_tasks_;

  std::vector<pal_wbc::COPContactFocesBoxConstraint6dDynamicPtr> cop_constraints_tasks_;
  // std::vector<pal_robot_tools::VectorReferenceAbstractPtr> cop_references_;
  // std::vector<pal_wbc::COPTracking6dDynamicPtr> cop_tracking_tasks_;
  std::vector<pal_wbc::FrictionConstraintDynamicTaskPtr> friction_constratin_tasks_;

  pal_robot_tools::VectorPointerReferencePtr angular_momentum_reference_;
  pal_wbc::AngularMomentumTargetDynamicTaskPtr angular_momentum_task_;

  std::vector<pal_wbc::GoToPositionDynamicMetaTaskPtr> go_to_end_effector_position_tasks_;
  std::vector<pal_wbc::GoToOrientationDynamicMetaTaskPtr> go_to_end_effector_orientation_tasks_;

  visualization_msgs::MarkerArray marray_;
  unsigned int index_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>> wbc_pub_;

  // Generic metatasks
  pal_wbc::GenericMetaTaskPtr constraint_metatasks_;
  pal_wbc::GenericMetaTaskPtr objective_metatasks_;

  pal_robot_tools::TimeProfilerPtr tp_;

  pal_statistics::RegistrationsRAII registered_variables_;

  // Contact descriptions
  std::vector<std::vector<unsigned int>> stance_leg_position_contact_description_ids_;
  std::vector<std::vector<unsigned int>> stance_leg_orientation_contact_descrption_ids_;

  // List of posible contact poinnts
  std::vector<pal_wbc::ContactDescription> contact_forces_description_;
  std::vector<pal_wbc::ContactDescription> contact_torques_description_;

  pal_robot_tools::FrictionConeGeneratorPtr fc_generator_;
  Eigen::MatrixXd friction_cone_basis_;


  // Actual configuration of the contact points
  std::vector<pal_wbc::ContactDescription> actual_contact_force_description_;
  std::vector<pal_wbc::ContactDescription> actual_contact_torque_description_;

  // Used to detect contact change
  std::vector<Side> prev_stance_leg_ids_;
  std::vector<Side> prev_swing_leg_ids_;
};
}
