#include <talos_pal_locomotion/walking_wbc_lower_body_torso_head.h>
#include <wbc_tasks/physics_tools.h>
#include <pal_ros_utils/visualization_tools.h>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <rbdl/addons/Utils.h>
#include <pal_ros_utils/reference/vector/vector_pointer_reference.h>
#include <pal_ros_utils/xmlrpc_helpers.h>
#include <pal_base_ros_controller/controller_utils.h>
#include <math_utils/eigen/type_conversions.h>
#include <pal_physics_utils/rbcomposite/urdf_model.h>
#include <pal_locomotion/biped_controller.h>

using namespace pal_wbc;
using namespace pal_robot_tools;

namespace pal_locomotion
{
WalkingWBCTalosLowerBodyTorsoHead::WalkingWBCTalosLowerBodyTorsoHead()
{
}

WalkingWBCTalosLowerBodyTorsoHead::~WalkingWBCTalosLowerBodyTorsoHead()
{
}


bool WalkingWBCTalosLowerBodyTorsoHead::configure(ros::NodeHandle &nh, const ros::Time &time,
                                                  const ros::Duration &dt, BipedParameters *bp,
                                                  const Eigen::VectorXd &initialQ,
                                                  const Eigen::VectorXd &initialQDot,
                                                  const std::vector<std::string> &footNames,
                                                  const bool &zeroComXYGain)
{
  upper_body_joint_names_.clear();
  upper_body_joint_names_.push_back("head_1_joint");
  upper_body_joint_names_.push_back("head_2_joint");
  upper_body_joint_names_.push_back("torso_1_joint");
  upper_body_joint_names_.push_back("torso_2_joint");

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


void WalkingWBCTalosLowerBodyTorsoHead::initializeTasks(ros::NodeHandle &nh, BipedParameters *bp,
                                                        const ros::Time &time,
                                                        const Eigen::VectorXd &initialQ)
{
  // Joint limit task
  {
    pal_wbc::JointPosVelAccLimitsDynamicTaskParameters param;

    param.joint_names_ = dynamic_stack_->getJointNames();

    pal::math_utils::copyVector(dynamic_stack_->getJointPositionLimitMin(), param.position_min_);
    pal::math_utils::copyVector(dynamic_stack_->getJointPositionLimitMax(), param.position_max_);
    param.position_time_parameter_ = 0.05;
    param.position_enable_recovery_ = true;
    param.position_min_[dynamic_stack_->getJointIndex("leg_left_4_joint")] = 0.3;
    param.position_min_[dynamic_stack_->getJointIndex("leg_right_4_joint")] = 0.3;


    pal::math_utils::copyVector(dynamic_stack_->getJointVelocityLimitMin(), param.velocity_min_);
    pal::math_utils::copyVector(dynamic_stack_->getJointVelocityLimitMax(), param.velocity_max_);
    param.velocity_time_parameter_ = 0.000;
    param.velocity_enable_recovery_ = false;
    param.velocity_clamp_ = true;

    param.acceleration_absmax_.setConstant(param.joint_names_.size(), 600);


    // -----
    for (const std::string &fixed_joint_name : upper_body_joint_names_)
    {
      param.joint_names_.erase(std::find(param.joint_names_.begin(),
                                         param.joint_names_.end(), fixed_joint_name));
    }

    Eigen::MatrixXd joint_selector;
    joint_selector.setZero(param.joint_names_.size(), dynamic_stack_->getJointNames().size());
    for (std::size_t i = 0; i < param.joint_names_.size(); ++i)
    {
      joint_selector(i, dynamic_stack_->getJointIndex(param.joint_names_[i])) = 1.0;
    }

    param.position_min_ = joint_selector * param.position_min_;
    param.position_max_ = joint_selector * param.position_max_;

    param.velocity_min_ = joint_selector * param.velocity_min_;
    param.velocity_max_ = joint_selector * param.velocity_max_;

    param.acceleration_absmax_ = joint_selector * param.acceleration_absmax_;
    // -----

    joint_position_limit_task_.reset(new JointPosVelAccLimitsDynamicTask(
        "joint_position_limit_task", param, dynamic_stack_.get(), nh));

    // Torque limits
    std::vector<double> torque_limits_all = dynamic_stack_->getJointTorqueLimits();
    std::vector<double> torque_limits_selected;

    for (const std::string &joint_name : param.joint_names_)
    {
      torque_limits_selected.push_back(
          torque_limits_all[dynamic_stack_->getJointIndex(joint_name)]);
    }

    torque_limits_task_.reset(new TorqueLimitDynamicAllJointsMetaTask(
        "torque_limits_task", dynamic_stack_.get(), torque_limits_selected, param.joint_names_,
        parameters_->torque_limit_factor_, nh, parameters_->friction_mu_));
  }

  {
    pal_wbc::JointPosVelAccLimitsDynamicTaskParameters param;
    param.joint_names_ = upper_body_joint_names_;

    param.acceleration_absmax_.setConstant(param.joint_names_.size(), 0);
    joint_position_limit_head_torso_task_.reset(new JointPosVelAccLimitsDynamicTask(
        "joint_position_limit_head_torso_task", param, dynamic_stack_.get(), nh));
  }

  WalkingWBCTalosBase::initializeTasks(nh, bp, time, initialQ);
}


void WalkingWBCTalosLowerBodyTorsoHead::updateTasks(
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


  constraint_tasks.push_back(joint_position_limit_head_torso_task_);
}


void WalkingWBCTalosLowerBodyTorsoHead::update(
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
    std::vector<std::pair<Eigen::Vector3d, ContactDescription> > &computed_forces,
    std::vector<std::pair<Eigen::Vector3d, ContactDescription> > &computed_torques)
{
  assert(Q.rows() == dynamic_stack_->getModel()->q_size);
  assert(QDot.rows() == dynamic_stack_->getNumberDofJointStateIncludingFloatingBase());
  assert(jointStateDesiredAcceleration.rows() == dynamic_stack_->getNumberDofJointState());
  assert(Tau.rows() == dynamic_stack_->getNumberDofJointState());

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


  //    std::cerr<<"*****"<<std::endl;
  //    std::cerr<<dynamic_stack_->printStack()<<std::endl;

  tp_->stopTime("wbc_setup");

  tp_->startTimer("wbc_task_update");
  std::vector<const Level *> levels;
  levels.reserve(100);  // allocation
  dynamic_stack_->updateTasks(Q, QDot, time);
  dynamic_stack_->getLevels(levels);
  tp_->stopTime("wbc_task_update");

  tp_->startTimer("wbc_optimization_qp");

  hqp_solver_->solve_hirarchy(levels, solution_);
  tp_->stopTime("wbc_optimization_qp");

  //  dynamic_stack_->debug(solution_, time);

  if (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced_friction_cone)
  {
    dynamic_stack_->publishFrictionConeForces(time, solution_, marray_, index_);
  }
  if (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced)
  {
    dynamic_stack_->publishForces(time, solution_, marray_, index_);
  }

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

PLUGINLIB_EXPORT_CLASS(pal_locomotion::WalkingWBCTalosLowerBodyTorsoHead,
                       pal_locomotion::BipedWbcBase);
