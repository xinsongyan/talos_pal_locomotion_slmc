/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#pragma once

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
#include <wbc_tasks/normal_torque_distribution_ds_dynamic_task.h>

#include <pal_ros_utils/reference/pose/pose_pointer_reference.h>
#include <pal_ros_utils/reference/vector/vector_pointer_reference.h>

#include <pal_locomotion/wbc/wbc_base.h>
#include <pluginlib/class_loader.h>
#include <hqp_solvers/hqp_solver.h>
#include <pal_locomotion/leg_trajectory/swing_leg_trajectory_base.h>

#include <pal_utils/time_profiler.h>

#include <pal_ros_utils/reference/vector/vector_reference_minjerk_dynamic_reconfigure.h>
#include <pal_ros_utils/reference/vector/vector_reference_dynamic_reconfigure.h>

#include <talos_pal_locomotion/walking_wbc_base.h>
#include <pal_locomotion/biped_controller.h>

namespace pal_locomotion
{
class WalkingWBCTalosLowerBodyTorsoHead : public WalkingWBCTalosBase
{
public:
  WalkingWBCTalosLowerBodyTorsoHead();

  virtual ~WalkingWBCTalosLowerBodyTorsoHead();

  virtual bool configure(ros::NodeHandle &nh, const ros::Time &time,
                         const ros::Duration &dt, BipedParameters *bp,
                         const Eigen::VectorXd &initialQ, const Eigen::VectorXd &initialQDot,
                         const std::vector<std::string> &footNames,
                         const bool &zeroComXYGain) override;

  void initializeTasks(ros::NodeHandle &nh, BipedParameters *bp, const ros::Time &time,
                       const Eigen::VectorXd &initialQ) override;

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


  virtual void update(
      const ros::Time &time, const std::vector<Side> stanceLegIDs,
      const std::vector<Side> swingLegIDs, const double &weightDistribution,
      const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot,
      const Eigen::Vector3d &comPosition, const Eigen::Vector3d &comVelocity,
      const Eigen::Vector3d &comAcceleration, const Eigen::Quaterniond &baseOrientation,
      const Eigen::Vector3d &baseAngularVelocity, const Eigen::Vector3d &baseAngularAcceleration,
      const Eigen::Quaterniond &torsoOrientation, const Eigen::Vector3d &torsoAngularVelocity,
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
      const std::vector<Eigen::Vector3d> &ee_angular_acceleration, Eigen::VectorXd &Tau,
      Eigen::Vector3d &floatingBaseDesiredLinearAcceleration,
      Eigen::Vector3d &floatingBaseDesiredAngularAcceleration,
      Eigen::VectorXd &jointStateDesiredAcceleration,
      std::vector<std::pair<Eigen::Vector3d, pal_wbc::ContactDescription>> &computed_forces,
      std::vector<std::pair<Eigen::Vector3d, pal_wbc::ContactDescription>> &computed_torques) override;

protected:
  /*
  pal_robot_tools::VectorDynamicReconfigureReferencePtr upper_body_joint_reference_;
  pal_wbc::ReferenceDynamicPostureTaskMetaTaskPtr reference_joint_task_;
  */

  pal_wbc::TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits_task_;
  pal_wbc::JointPosVelAccLimitsDynamicTaskPtr joint_position_limit_task_;
  pal_wbc::JointPosVelAccLimitsDynamicTaskPtr joint_position_limit_head_torso_task_;
};
}
