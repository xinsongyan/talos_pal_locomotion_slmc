#include <Eigen/Dense>
#include <chrono>
#include <math_utils/eigen/type_conversions.h>
#include <pal_base_ros_controller/controller_utils.h>
#include <pal_physics_utils/rbcomposite/urdf_model.h>
#include <pal_ros_utils/reference/vector/vector_pointer_reference.h>
#include <pal_ros_utils/visualization_tools.h>
#include <pal_ros_utils/xmlrpc_helpers.h>
#include <pluginlib/class_list_macros.h>
#include <rbdl/addons/Utils.h>
#include <talos_pal_locomotion/wbc_weighted_dynamics_talos.h>
#include <wbc_tasks/physics_tools.h>

using namespace pal_wbc;
using namespace pal_robot_tools;

namespace pal_locomotion
{
WbcWeightedDynamicCopTALOS::WbcWeightedDynamicCopTALOS()
{
}

WbcWeightedDynamicCopTALOS::~WbcWeightedDynamicCopTALOS()
{
}

bool WbcWeightedDynamicCopTALOS::configure(ros::NodeHandle &nh, const ros::Time &time,
                                           const ros::Duration &dt, BipedParameters *bp,
                                           const Eigen::VectorXd &initialQ,
                                           const Eigen::VectorXd &initialQDot,
                                           const std::vector<std::string> &footNames,
                                           const bool &zeroComXYGain)
{
  upper_body_joint_names_.push_back("arm_left_1_joint");
  upper_body_joint_names_.push_back("arm_left_2_joint");
  upper_body_joint_names_.push_back("arm_left_3_joint");
  upper_body_joint_names_.push_back("arm_left_4_joint");
  upper_body_joint_names_.push_back("arm_left_5_joint");
  upper_body_joint_names_.push_back("arm_left_6_joint");
  upper_body_joint_names_.push_back("arm_left_7_joint");
  upper_body_joint_names_.push_back("arm_right_1_joint");
  upper_body_joint_names_.push_back("arm_right_2_joint");
  upper_body_joint_names_.push_back("arm_right_3_joint");
  upper_body_joint_names_.push_back("arm_right_4_joint");
  upper_body_joint_names_.push_back("arm_right_5_joint");
  upper_body_joint_names_.push_back("arm_right_6_joint");
  upper_body_joint_names_.push_back("arm_right_7_joint");
  upper_body_joint_names_.push_back("head_1_joint");
  upper_body_joint_names_.push_back("head_2_joint");

  gripper_joint_names_.push_back("gripper_left_joint");
  gripper_joint_names_.push_back("gripper_right_joint");

  pal::rbcomposite::URDFModel::getDefaultConfiguration(
      nh, "/zeros", upper_body_joint_names_, upper_body_joint_default_configuration_);

  if (true == WalkingWBCTalosBase::configure(nh, time, dt, bp, initialQ, initialQDot,
                                             footNames, zeroComXYGain))
  {
    initializeTasks(nh, bp, time, initialQ);
    return (true);
  }
  else
  {
    return (false);
  }
}

void WbcWeightedDynamicCopTALOS::initializeTasks(ros::NodeHandle &nh, BipedParameters *bp,
                                                 const ros::Time &time,
                                                 const Eigen::VectorXd &initialQ)
{
  std::vector<double> upper_body_joint_min(upper_body_joint_default_configuration_.rows());
  std::vector<double> upper_body_joint_max(upper_body_joint_default_configuration_.rows());
  std::vector<double> joint_min = dynamic_stack_->getJointPositionLimitMin();
  std::vector<double> joint_max = dynamic_stack_->getJointPositionLimitMax();
  for (size_t i = 0; i < upper_body_joint_min.size(); ++i)
  {
    upper_body_joint_min[i] =
        joint_min[dynamic_stack_->getJointIndex(upper_body_joint_names_[i])];
    upper_body_joint_max[i] =
        joint_max[dynamic_stack_->getJointIndex(upper_body_joint_names_[i])];
  }

  //  upper_body_joint_reference_.reset(
  //      new pal_robot_tools::VectorMinJerkDynamicReconfigureReference(
  //          nh, "upper_body_joint_reference", upper_body_joint_names_.size(),
  //          upper_body_joint_names_, upper_body_joint_min,
  //          upper_body_joint_max,
  //          upper_body_joint_default_configuration_,
  //          upper_body_joint_default_configuration_));

  upper_body_joint_reference_.reset(new pal_robot_tools::VectorPointerReference(
      nh, upper_body_joint_names_.size(), "upper_body_joint_reference"));

  //  // grippers
  //  {
  //    std::vector<double> grippers_joint_min(gripperJointNames_.size());
  //    std::vector<double> grippers_joint_max(gripperJointNames_.size());
  //    Eigen::VectorXd initial_configuration;
  //    pal::rbcomposite::URDFModel::getDefaultConfiguration(nh, "/zeros",
  //    gripperJointNames_,
  //                                                         initial_configuration);
  //    for (size_t i = 0; i < gripperJointNames_.size(); ++i)
  //    {
  //      grippers_joint_min[i] =
  //      joint_min[dynamic_stack_->getJointIndex(gripperJointNames_[i])];
  //      grippers_joint_max[i] =
  //      joint_max[dynamic_stack_->getJointIndex(gripperJointNames_[i])];
  //    }

  //    grippers_joint_reference_.reset(new
  //    pal_robot_tools::VectorDynamicReconfigureReference(
  //        nh, "grippers_joint_reference", gripperJointNames_.size(),
  //        gripperJointNames_,
  //        grippers_joint_min, grippers_joint_max, initial_configuration));

  //    reference_joint_grippers_task_.reset(new
  //    ReferenceDynamicPostureTaskMetaTask(
  //        "reference_gripper_joints", dynamic_stack_.get(),
  //        gripperJointNames_,
  //        grippers_joint_reference_, 200, nh));
  //  }

  wbc_gains_->upper_body_joint_reference_gains_.reset(
      new pal_ros_utils::PIDRateLimitedParameters("upper_body_joint_gains", nh));
  wbc_gains_->upper_body_joint_reference_gains_->p_gain_ =
      parameters_->upper_body_joint_reference_p_gain_;

  upper_body_joint_reference_->setPositionTarget(upper_body_joint_default_configuration_);
  reference_joint_task_.reset(new ReferenceDynamicPostureTaskMetaTask(
      "reference_upper_body_joints", dynamic_stack_.get(), upper_body_joint_names_,
      upper_body_joint_reference_, wbc_gains_->upper_body_joint_reference_gains_, nh));
  reference_joint_task_->setWeight(parameters_->upper_body_joint_reference_weight_);

  // Joint limit task
  pal_wbc::JointPosVelAccLimitsDynamicTaskParameters param;

  param.joint_names_ = dynamic_stack_->getJointNames();

  pal::math_utils::copyVector(dynamic_stack_->getJointPositionLimitMin(), param.position_min_);
  pal::math_utils::copyVector(dynamic_stack_->getJointPositionLimitMax(), param.position_max_);
  param.position_time_parameter_ = 0.01;
  param.position_enable_recovery_ = false;
  param.position_min_[dynamic_stack_->getJointIndex("leg_left_4_joint")] = 0.3;
  param.position_min_[dynamic_stack_->getJointIndex("leg_right_4_joint")] = 0.3;

  pal::math_utils::copyVector(dynamic_stack_->getJointVelocityLimitMin(), param.velocity_min_);
  pal::math_utils::copyVector(dynamic_stack_->getJointVelocityLimitMax(), param.velocity_max_);
  param.velocity_time_parameter_ = 0.0;     // 0 -> disable velocity limits
  param.velocity_enable_recovery_ = false;  // not relevant while disabled
  param.velocity_clamp_ = true;

  param.acceleration_absmax_.setConstant(param.joint_names_.size(), 1000);

  joint_position_limit_task_.reset(new JointPosVelAccLimitsDynamicTask(
      "joint_position_limit_task", param, dynamic_stack_.get(), nh));

  // Torque limits
  torque_limits_task_.reset(new TorqueLimitDynamicAllJointsMetaTask(
      "torque_limits_task", dynamic_stack_.get(), dynamic_stack_->getJointTorqueLimits(),
      dynamic_stack_->getJointNames(), parameters_->torque_limit_factor_, nh,
      parameters_->friction_mu_));

  {
    pal_wbc::JointPosVelAccLimitsDynamicTaskParameters param2;
    param2.joint_names_ = gripper_joint_names_;

    param2.acceleration_absmax_.setConstant(param2.joint_names_.size(), 0);
    grippers_limit_task_.reset(new JointPosVelAccLimitsDynamicTask(
        "joint_position_limit_torso_task", param2, dynamic_stack_.get(), nh));
  }

  // End-effector tasks
  {
    wbc_gains_->ee_position_gains_.reset(
        new pal_ros_utils::PIDRateLimitedParameters("ee_position_gains", nh));
    wbc_gains_->ee_position_gains_->p_gain_ = parameters_->end_effector_position_p_gain_;

    wbc_gains_->ee_orientation_gains_.reset(
        new pal_ros_utils::PIDRateLimitedParameters("ee_orientation_gains", nh));
    wbc_gains_->ee_orientation_gains_->p_gain_ = parameters_->end_effector_orientation_p_gain_;

    RigidBodyDynamics::Model *robot_model = dynamic_stack_->getModel();

    end_effector_names_ = { bp->left_ee_name_, bp->right_ee_name_ };
    for (size_t i = 0; i < end_effector_names_.size(); ++i)
    {
      unsigned int body_id = robot_model->GetBodyId(end_effector_names_[i].c_str());
      Eigen::Vector3d ee_position = RigidBodyDynamics::CalcBodyToBaseCoordinates(
          *robot_model, initialQ, body_id, Eigen::Vector3d::Zero(), false);

      Eigen::Matrix3d ee_orientation =
          RigidBodyDynamics::CalcBodyWorldOrientation(*robot_model, initialQ, body_id, false)
              .transpose();

      PointerReferencePtr ee_target(new PointerReference(nh, end_effector_names_[i],
                                                         "world", "world", ee_position,
                                                         eQuaternion(ee_orientation)));
      ee_targets_.push_back(ee_target);

      GoToPositionDynamicMetaTaskPtr go_to_ee_position_task(new GoToPositionDynamicMetaTask(
          "go_to_position_" + end_effector_names_[i], dynamic_stack_.get(),
          end_effector_names_[i], eVector3::Zero(), ee_target, nh,
          wbc_gains_->ee_position_gains_));
      go_to_ee_position_task->setWeight(parameters_->end_effector_position_weight_);
      go_to_end_effector_position_tasks_.push_back(go_to_ee_position_task);

      GoToOrientationDynamicMetaTaskPtr go_to_ee_orientation_task(new GoToOrientationDynamicMetaTask(
          "go_to_orientation_" + end_effector_names_[i], dynamic_stack_.get(),
          end_effector_names_[i], nh, ee_target, wbc_gains_->ee_orientation_gains_));
      go_to_ee_orientation_task->setWeight(parameters_->end_effector_orientation_weight_);
      go_to_end_effector_orientation_tasks_.push_back(go_to_ee_orientation_task);
    }
  }

  WalkingWBCTalosBase::initializeTasks(nh, bp, time, initialQ);
}

void WbcWeightedDynamicCopTALOS::updateTasks(
    const ros::Time &time, const std::vector<Side> stanceLegIDs,
    const std::vector<Side> swingLegIDs, const double &weightDistribution,
    const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot, const Eigen::Vector3d &comPosition,
    const Eigen::Vector3d &comVelocity, const Eigen::Vector3d &comAcceleration,
    const Eigen::Quaterniond &baseOrientation, const Eigen::Vector3d &baseAngularVelocity,
    const Eigen::Vector3d &baseAngularAcceleration, const Eigen::Quaterniond &torsoOrientation,
    const Eigen::Vector3d &torsoAngularVelocity, const Eigen::Vector3d &torsoAngularAcceleration,
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
    const std::vector<Eigen::Vector3d> &ee_position, const std::vector<eQuaternion> &ee_orientation,
    const std::vector<Eigen::Vector3d> &ee_linear_velocity,
    const std::vector<Eigen::Vector3d> &ee_angular_velocity,
    const std::vector<Eigen::Vector3d> &ee_linear_acceleration,
    const std::vector<Eigen::Vector3d> &ee_angular_acceleration,
    task_container_vector &constraint_tasks, task_container_vector &objective_tasks)
{
  PAL_ASSERT_PERSIST_EQUAL(stanceLegIDs.size() + swingLegIDs.size(), 2);

  WalkingWBCTalosBase::updateTasks(
      time, stanceLegIDs, swingLegIDs, weightDistribution, Q, QDot, comPosition, comVelocity,
      comAcceleration, baseOrientation, baseAngularVelocity, baseAngularAcceleration,
      torsoOrientation, torsoAngularVelocity, torsoAngularAcceleration,
      stance_leg_positions, stance_leg_orientations, swingLegPositions,
      swingLegLinearVelocitys, swingLegLinearAcceleartions, swingLegOrientations,
      swingLegAngularVelocitys, swingLegAngularAcceleartions, desiredUpperBodyJointPositions,
      desiredUpperBodyJointVelocitys, desiredUpperBodyJointAccelerations,
      desiredAngularMomentum, desiredAngularMomentumVelocity, ee_position, ee_orientation,
      ee_linear_velocity, ee_angular_velocity, ee_linear_acceleration,
      ee_angular_acceleration, constraint_tasks, objective_tasks);

  //// Objectives
  upper_body_joint_reference_->setPositionTarget(desiredUpperBodyJointPositions);
  upper_body_joint_reference_->setVelocityTarget(desiredUpperBodyJointVelocitys);
  upper_body_joint_reference_->setAccelerationTarget(desiredUpperBodyJointAccelerations);

  objective_tasks.push_back(orientation_upper_torso_task_);
  objective_tasks.push_back(reference_joint_task_);

  constraint_tasks.push_back(grippers_limit_task_);

  if (parameters_->enable_joint_position_limits_)
  {
    constraint_tasks.push_back(joint_position_limit_task_);
  }

  if (parameters_->enable_joint_torque_limits_)
  {
    constraint_tasks.push_back(torque_limits_task_);
  }

  PAL_ASSERT_PERSIST_EQUAL(end_effector_names_.size(), 2);
  PAL_ASSERT_PERSIST_EQUAL(go_to_end_effector_position_tasks_.size(), 2);
  PAL_ASSERT_PERSIST_EQUAL(go_to_end_effector_orientation_tasks_.size(), 2);

  for (size_t i = 0; i < end_effector_names_.size(); ++i)
  {
    ee_targets_[i]->setPositionTarget(ee_position[i]);
    ee_targets_[i]->setOrientationTarget(ee_orientation[i]);
    ee_targets_[i]->setLinearVelocityTarget(ee_linear_velocity[i]);
    ee_targets_[i]->setAngularVelocityTarget(ee_angular_velocity[i]);
    ee_targets_[i]->setLinearAccelerationTarget(ee_linear_acceleration[i]);
    ee_targets_[i]->setAngularAccelerationTarget(ee_angular_acceleration[i]);

    objective_tasks.push_back(go_to_end_effector_position_tasks_[i]);
    go_to_end_effector_position_tasks_[i]->setWeight(parameters_->end_effector_position_weight_);
    objective_tasks.push_back(go_to_end_effector_orientation_tasks_[i]);
    go_to_end_effector_orientation_tasks_[i]->setWeight(parameters_->end_effector_orientation_weight_);
  }
  //  // grippers
  //  {
  //    objective_tasks.push_back(reference_joint_grippers_task_);
  //  }
}

void WbcWeightedDynamicCopTALOS::update(
    const ros::Time &time, const std::vector<Side> stance_leg_ids,
    const std::vector<Side> swing_leg_ids, const double &weightDistribution,
    const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot, const Eigen::Vector3d &comPosition,
    const Eigen::Vector3d &comVelocity, const Eigen::Vector3d &comAcceleration,
    const Eigen::Quaterniond &baseOrientation, const Eigen::Vector3d &baseAngularVelocity,
    const Eigen::Vector3d &baseAngularAcceleration, const Eigen::Quaterniond &torsoOrientation,
    const Eigen::Vector3d &torsoAngularVelocity, const Eigen::Vector3d &torsoAngularAcceleration,
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
    const std::vector<Eigen::Vector3d> &ee_position, const std::vector<eQuaternion> &ee_orientation,
    const std::vector<Eigen::Vector3d> &ee_linear_velocity,
    const std::vector<Eigen::Vector3d> &ee_angular_velocity,
    const std::vector<Eigen::Vector3d> &ee_linear_acceleration,
    const std::vector<Eigen::Vector3d> &ee_angular_acceleration, Eigen::VectorXd &Tau,
    Eigen::Vector3d &floatingBaseDesiredLinearAcceleration,
    Eigen::Vector3d &floatingBaseDesiredAngularAcceleration,
    Eigen::VectorXd &jointStateDesiredAcceleration,
    std::vector<std::pair<Eigen::Vector3d, ContactDescription>> &computed_forces,
    std::vector<std::pair<Eigen::Vector3d, ContactDescription>> &computed_torques)
{
  PAL_ASSERT_PERSIST_EQUAL(Q.rows(), dynamic_stack_->getModel()->q_size);
  PAL_ASSERT_PERSIST_EQUAL(QDot.rows(),
                           dynamic_stack_->getNumberDofJointStateIncludingFloatingBase());
  PAL_ASSERT_PERSIST_EQUAL(jointStateDesiredAcceleration.rows(),
                           dynamic_stack_->getNumberDofJointState());
  PAL_ASSERT_PERSIST_EQUAL(Tau.rows(), dynamic_stack_->getNumberDofJointState());

  tp_->startTimer("wbc_setup");

  task_container_vector constraint_tasks;
  task_container_vector objective_tasks;

  updateTasks(time, stance_leg_ids, swing_leg_ids, weightDistribution, Q, QDot, comPosition,
              comVelocity, comAcceleration, baseOrientation, baseAngularVelocity,
              baseAngularAcceleration, torsoOrientation, torsoAngularVelocity,
              torsoAngularAcceleration, stance_leg_positions, stance_leg_orientations,
              swingLegPositions, swingLegLinearVelocitys, swingLegLinearAcceleartions,
              swingLegOrientations, swingLegAngularVelocitys, swingLegAngularAcceleartions,
              desiredUpperBodyJointPositions, desiredUpperBodyJointVelocitys,
              desiredUpperBodyJointAccelerations, desiredAngularMomentum,
              desiredAngularMomentumVelocity, ee_position, ee_orientation,
              ee_linear_velocity, ee_angular_velocity, ee_linear_acceleration,
              ee_angular_acceleration, constraint_tasks, objective_tasks);

  bool contacts_changed = false;
  if (stance_leg_ids.size() != prev_stance_leg_ids_.size())
  {
    contacts_changed = true;
    prev_stance_leg_ids_ = stance_leg_ids;
  }
  if (swing_leg_ids.size() != prev_swing_leg_ids_.size())
  {
    contacts_changed = true;
    prev_swing_leg_ids_ = swing_leg_ids;
  }

  if (contacts_changed || dynamic_stack_->getNumberOfTasks() == 0)
  {
    dynamic_stack_->popAllTasks();

    for (size_t i = 0; i < constraint_tasks.size(); ++i)
    {
      constraint_tasks[i]->reconfigureTask();
    }

    for (size_t i = 0; i < objective_tasks.size(); ++i)
    {
      objective_tasks[i]->reconfigureTask();
    }

    constraint_metatasks_->reconfigureTask(constraint_tasks);
    objective_metatasks_->reconfigureTask(objective_tasks);

    dynamic_stack_->pushTask(constraint_metatasks_);
    dynamic_stack_->pushTask(objective_metatasks_);
  }

  tp_->stopTime("wbc_setup");

  tp_->startTimer("wbc_task_update");
  std::vector<const Level *> levels;
  levels.reserve(100);  // allocation
  dynamic_stack_->updateTasks(Q, QDot, time);
  dynamic_stack_->getLevels(levels);
  tp_->stopTime("wbc_task_update");

  //  std::cerr<<dynamic_stack_->printStack()<<std::endl;
  //  std::cerr<<"**********"<<std::endl;

  tp_->startTimer("wbc_optimization_qp");

  hqp_solver_->solve_hirarchy(levels, solution_);
  tp_->stopTime("wbc_optimization_qp");

  //  dynamic_stack_->debug(solution_, time);

  // dynamic_stack_->publishFrictionConeForces(time, solution_, marray_,
  // index_);

  if (wbc_pub_->trylock())
  {
    wbc_pub_->msg_ = marray_;
    wbc_pub_->unlockAndPublish();
  }
  marray_.markers.clear();
  index_ = 0;

  WalkingWBCTalosBase::processSolution(
      stance_leg_ids, Q, QDot, Tau, floatingBaseDesiredLinearAcceleration,
      floatingBaseDesiredAngularAcceleration, jointStateDesiredAcceleration,
      computed_forces, computed_torques);
}
}

PLUGINLIB_EXPORT_CLASS(pal_locomotion::WbcWeightedDynamicCopTALOS, pal_locomotion::BipedWbcBase);
