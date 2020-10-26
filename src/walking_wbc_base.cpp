#include <talos_pal_locomotion/wbc_weighted_dynamics_talos.h>
#include <wbc_tasks/physics_tools.h>
#include <pal_ros_utils/visualization_tools.h>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <rbdl/addons/Utils.h>
#include <pal_ros_utils/reference/vector/vector_pointer_reference.h>
#include <pal_ros_utils/reference/pose/pose_interactive_marker_reference.h>
#include <pal_ros_utils/xmlrpc_helpers.h>
#include <pal_base_ros_controller/controller_utils.h>
#include <math_utils/eigen/type_conversions.h>
#include <rbdl/Model.h>

using namespace pal_ros_utils;
using namespace pal_wbc;
using namespace pal_robot_tools;

namespace pal_locomotion
{
WalkingWBCTalosBase::~WalkingWBCTalosBase()
{
}

bool WalkingWBCTalosBase::configure(ros::NodeHandle &nh, const ros::Time &time,
                                    const ros::Duration &dt, BipedParameters *bc,
                                    const Eigen::VectorXd &initialQ,
                                    const Eigen::VectorXd &initialQDot,
                                    const std::vector<std::string> &foot_names,
                                    const bool &zeroComXYGain)
{
  parameters_->readConfig<ariles::ros>(nh, "wbc_parameters");
  wbc_gains_.reset(new WBCGains());

  prev_stance_leg_ids_.push_back(Side::LEFT);
  prev_stance_leg_ids_.push_back(Side::RIGHT);

  FootDescription foot_description = bc->foot_description_;

  index_ = 0.;
  marray_.markers.reserve(1000);
  wbc_pub_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>(
      nh, "wbc_markers", 1));

  foot_names_ = foot_names;
  dt_ = dt;
  zeroComXYGain_ = zeroComXYGain;

  tp_.reset(new pal_robot_tools::TimeProfiler);
  tp_->registerTimer("wbc_setup");
  tp_->registerTimer("wbc_task_update");
  tp_->registerTimer("wbc_optimization_qp");

  REGISTER_VARIABLE("/introspection_data", "wbc_setup_elapsed_time",
                    tp_->getLastCycleTimePtr("wbc_setup"), &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "wbc_task_update_elapsed_time",
                    tp_->getLastCycleTimePtr("wbc_task_update"), &registered_variables_);
  REGISTER_VARIABLE("/introspection_data", "wbc_optimization_qp_elapsed_time",
                    tp_->getLastCycleTimePtr("wbc_optimization_qp"), &registered_variables_);

  hqp_solver_loader_.reset(new pluginlib::ClassLoader<HQPSolver>("hqp_solvers", "HQPSolver"));

  OptimizationParameters opt_parameters;
  opt_parameters.readConfig<ariles::ros>(nh, "optimization_parameters");
  try
  {
    hqp_solver_ = hqp_solver_loader_->createInstance(opt_parameters.solver_);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  if (!hqp_solver_)
  {
    ROS_ERROR_STREAM("Error creating hqp_solver");
  }
  else
  {
    ROS_DEBUG_STREAM("Succes creating the solver");
  }

  switch (formulation_t::_from_string(parameters_->formulation_.c_str()))
  {
    case formulation_t::dynamics:
    case formulation_t::dynamics_reduced:
    {
      // Initialize double support
      unsigned int force_index = 0;
      unsigned int torque_index = 0.;
      stance_leg_position_contact_description_ids_.resize(2);
      stance_leg_orientation_contact_descrption_ids_.resize(2);

      for (size_t i = 0; i < 2; ++i)
      {
        contact_forces_description_.push_back(
            std::pair<std::string, Eigen::Vector3d>(foot_names_[i], Eigen::Vector3d::Zero()));
        stance_leg_position_contact_description_ids_[i].push_back(force_index);
        ++force_index;
        contact_torques_description_.push_back(
            std::pair<std::string, Eigen::Vector3d>(foot_names_[i], Eigen::Vector3d::Zero()));
        stance_leg_orientation_contact_descrption_ids_[i].push_back(torque_index);
        ++torque_index;
      }
    }
    break;

    case formulation_t::dynamics_reduced_friction_cone:
    {
      fc_generator_.reset(new FrictionConeGenerator(parameters_->friction_mu_));
      friction_cone_basis_ = fc_generator_->getMatrixForm();


      // Initialize double support
      unsigned int force_index = 0;
      stance_leg_position_contact_description_ids_.resize(2);
      stance_leg_orientation_contact_descrption_ids_.resize(2);

      for (size_t i = 0; i < 2; ++i)
      {
        contact_forces_description_.push_back(std::pair<std::string, Eigen::Vector3d>(
            foot_names_[i], Eigen::Vector3d(foot_description.foot_length_ / 2.,
                                            foot_description.foot_with_ / 2., 0.)));
        stance_leg_position_contact_description_ids_[i].push_back(force_index);
        ++force_index;

        contact_forces_description_.push_back(std::pair<std::string, Eigen::Vector3d>(
            foot_names_[i], Eigen::Vector3d(foot_description.foot_length_ / 2.,
                                            -foot_description.foot_with_ / 2., 0.)));
        stance_leg_position_contact_description_ids_[i].push_back(force_index);
        ++force_index;

        contact_forces_description_.push_back(std::pair<std::string, Eigen::Vector3d>(
            foot_names_[i], Eigen::Vector3d(-foot_description.foot_length_ / 2.,
                                            foot_description.foot_with_ / 2., 0.)));
        stance_leg_position_contact_description_ids_[i].push_back(force_index);
        ++force_index;

        contact_forces_description_.push_back(std::pair<std::string, Eigen::Vector3d>(
            foot_names_[i], Eigen::Vector3d(-foot_description.foot_length_ / 2.,
                                            -foot_description.foot_with_ / 2., 0.)));
        stance_leg_position_contact_description_ids_[i].push_back(force_index);
        ++force_index;
      }
    }
    break;

    default:
      PAL_THROW_DEFAULT("not supported");
  }

  std::vector<std::string> subtree_tips;
  pal_base_ros_controller::parseSubTreeTips(nh, subtree_tips);

  /// @todo Dynamics friction cone breaks the physics
  dynamic_stack_.reset(new StackOfTasksDynamic(
      nh, formulation_t::_from_string(parameters_->formulation_.c_str()), solution_, dt_,
      RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion, contact_forces_description_,
      contact_torques_description_, subtree_tips, parameters_->friction_mu_));

  if (nh.hasParam("com_offset_parameters"))
  {
    COMOffsetHelperParameters com_offset_parameters;
    com_offset_parameters.readConfig<ariles::ros>(nh, "com_offset_parameters");
    com_offset_helper_.reset(
        new COMOffsetHelper(nh, dynamic_stack_->getModel(), com_offset_parameters));
  }

  dynamic_stack_->updateTasks(initialQ, initialQDot, time);

  generalized_acceleration_.setZero(dynamic_stack_->getModel()->qdot_size);

  configured_ = true;

  return true;
}


void WalkingWBCTalosBase::initializeTasks(ros::NodeHandle &nh, BipedParameters *bp,
                                          const ros::Time &time, const Eigen::VectorXd &initialQ)
{
  FootDescription foot_description = bp->foot_description_;

  base_target_.reset(new PointerReference(nh, "base_link", "world", "world",
                                          eVector3(0, 0, 0), eQuaternion::Identity()));
  torso_target_.reset(new PointerReference(nh, "torso_2_link", "world", "world",
                                           eVector3(0, 0, 0), eQuaternion::Identity()));

  RigidBodyDynamics::Model *robotModel = dynamic_stack_->getModel();
  for (size_t i = 0; i < foot_names_.size(); ++i)
  {
    unsigned int body_id = robotModel->GetBodyId(foot_names_[i].c_str());
    Eigen::Vector3d footPosition = RigidBodyDynamics::CalcBodyToBaseCoordinates(
        *robotModel, initialQ, body_id, Eigen::Vector3d::Zero(), false);

    Eigen::Matrix3d footOrientation =
        RigidBodyDynamics::CalcBodyWorldOrientation(*robotModel, initialQ, body_id, false).transpose();

    PointerReferencePtr foot_target(
        new PointerReference(nh, foot_names_[i], "world", "world", footPosition,
                             Eigen::Quaterniond(footOrientation)));
    swing_leg_targets_.push_back(foot_target);
  }

  // Physics
  // Unilateral contact foces == ZMP in flat terrain
  {
    unilateral_forces_task_.reset(new UnilateralForcesDynamics());
    UnilateralForcesParams unilateral_forces_params;
    unilateral_forces_params.minium_value_ = parameters_->minium_unilateral_force_;
    if (!unilateral_forces_task_->setUp("unilateral_forces", unilateral_forces_params,
                                        dynamic_stack_.get(), nh))
    {
      PAL_THROW_DEFAULT("problem configuring unilateral forces task");
    }
  }

  dynamic_task_.reset(new PhysicsDynamicConstraintMetaTask("dynamics", dynamic_stack_.get(),
                                                           nh, parameters_->friction_mu_));

  {
    ariles::ConfigurableFlags wbc_gains_flags;
    /// @todo AS: missing entries should not be allowed.
    wbc_gains_flags.set(ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES |
                        ariles::ConfigurableFlags::PROPAGATE_ALLOW_MISSING_ENTRIES);

    ros::NodeHandle wbc_gains_nh(nh, "wbc_parameters");

    wbc_gains_->stance_foot_position_xy_gains_ =
        boost::make_shared<pal_ros_utils::PIDRateLimitedParameters>(
            "stance_foot_position_xy_gains", nh);
    wbc_gains_->stance_foot_position_xy_gains_->readConfig<ariles::ros>(
        wbc_gains_nh, "stance_foot_position_xy_gains", wbc_gains_flags);

    wbc_gains_->stance_foot_position_z_gains_ =
        boost::make_shared<pal_ros_utils::PIDRateLimitedParameters>(
            "stance_foot_position_z_gains", nh);
    wbc_gains_->stance_foot_position_z_gains_->readConfig<ariles::ros>(
        wbc_gains_nh, "stance_foot_position_z_gains", wbc_gains_flags);

    wbc_gains_->stance_foot_orientation_roll_pitch_gains_ =
        boost::make_shared<pal_ros_utils::PIDRateLimitedParameters>(
            "stance_foot_orientation_roll_pitch_gains", nh);
    wbc_gains_->stance_foot_orientation_roll_pitch_gains_->readConfig<ariles::ros>(
        wbc_gains_nh, "stance_foot_orientation_roll_pitch_gains", wbc_gains_flags);

    wbc_gains_->stance_foot_orientation_yaw_gains_ =
        boost::make_shared<pal_ros_utils::PIDRateLimitedParameters>(
            "stance_foot_orientation_yaw_gains", nh);
    wbc_gains_->stance_foot_orientation_yaw_gains_->readConfig<ariles::ros>(
        wbc_gains_nh, "stance_foot_orientation_yaw_gains", wbc_gains_flags);
  }



  for (size_t i = 0; i < foot_names_.size(); ++i)
  {
    unsigned int body_id = robotModel->GetBodyId(foot_names_[i].c_str());
    Eigen::Vector3d footPosition = RigidBodyDynamics::CalcBodyToBaseCoordinates(
        *robotModel, initialQ, body_id, Eigen::Vector3d::Zero(), false);

    Eigen::Matrix3d footOrientation =
        RigidBodyDynamics::CalcBodyWorldOrientation(*robotModel, initialQ, body_id, false).transpose();
    PointerReferencePtr foot_target(
        new PointerReference(nh, foot_names_[i], "world", "world", footPosition,
                             Eigen::Quaternion<double>(footOrientation)));
    stance_leg_targets_.push_back(foot_target);
  }

  // We will get the contact configuration to setup the physics and constraints
  // from what is stored in the stack
  switch (parameters_->physics_parameters_.contact_task_type_)
  {
    case ContactTaskType::CONTACT_TASK_CARTESIAN_IMPEDANCE:

      for (unsigned int i = 0; i < foot_names_.size(); ++i)
      {
        GoToPositionDynamicMetaTaskPtr constraint_task_xy(new GoToPositionDynamicMetaTask(
            "position_constraint_xy_" + foot_names_[i], dynamic_stack_.get(),
            foot_names_[i], eVector3::Zero(), stance_leg_targets_[i], nh,
            wbc_gains_->stance_foot_position_xy_gains_, { coord_t::X, coord_t::Y }));
        position_constraint_xy_tasks_.push_back(constraint_task_xy);

        GoToPositionDynamicMetaTaskPtr constraint_task_z(new GoToPositionDynamicMetaTask(
            "position_constraint_z_" + foot_names_[i], dynamic_stack_.get(),
            foot_names_[i], eVector3::Zero(), stance_leg_targets_[i], nh,
            wbc_gains_->stance_foot_position_z_gains_, { coord_t::Z }));
        position_constraint_z_tasks_.push_back(constraint_task_z);
      }

      for (unsigned int i = 0; i < foot_names_.size(); ++i)
      {
        GoToOrientationDynamicMetaTaskPtr constraint_task_roll_pitch((new GoToOrientationDynamicMetaTask(
            "orientation_roll_pitch_constraint_" + foot_names_[i], dynamic_stack_.get(),
            foot_names_[i], nh, stance_leg_targets_[i], wbc_gains_->stance_foot_orientation_roll_pitch_gains_,
            { coord_t::Roll, coord_t::Pitch })));
        orientation_constraint_roll_pitch_tasks_.push_back(constraint_task_roll_pitch);

        GoToOrientationDynamicMetaTaskPtr constraint_task_yaw((new GoToOrientationDynamicMetaTask(
            "orientation_yaw_constraint_" + foot_names_[i], dynamic_stack_.get(),
            foot_names_[i], nh, stance_leg_targets_[i],
            wbc_gains_->stance_foot_orientation_yaw_gains_, { coord_t::Yaw })));
        orientation_constraint_yaw_tasks_.push_back(constraint_task_yaw);
      }

      break;

    default:
      PAL_THROW("Contact constraint type is not supported");
  }

  // COP constraint
  if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced_friction_cone))
  {
    PAL_ASSERT_PERSIST_BIGGER_OR_EQUAL(parameters_->cop_margin_x_, 0.);
    PAL_ASSERT_PERSIST_LESS(parameters_->cop_margin_x_, foot_description.foot_length_ / 2.);

    PAL_ASSERT_PERSIST_BIGGER_OR_EQUAL(parameters_->cop_margin_y_, 0.);
    PAL_ASSERT_PERSIST_LESS(parameters_->cop_margin_y_, foot_description.foot_with_ / 2.);

    for (size_t i = 0; i < 2; ++i)
    {
      COPContactFocesBoxConstraint6dParam dynamic_params;
      dynamic_params.footFrame = foot_names_[i];
      dynamic_params.minDx = -(foot_description.foot_length_ / 2. - parameters_->cop_margin_x_);
      dynamic_params.maxDx = foot_description.foot_length_ / 2. - parameters_->cop_margin_x_;
      dynamic_params.minDy = -(foot_description.foot_with_ / 2. - parameters_->cop_margin_y_);
      dynamic_params.maxDy = foot_description.foot_with_ / 2. - parameters_->cop_margin_y_;
      dynamic_params.mu_ = parameters_->friction_mu_;

      dynamic_params.torsion_friction_coef_ = parameters_->torsion_friction_coef_;

      COPContactFocesBoxConstraint6dDynamicPtr cop_constraint(
          new COPContactFocesBoxConstraint6dDynamic());
      if (!cop_constraint->setUp("cop_constraint_" + foot_names_[i], dynamic_params,
                                 dynamic_stack_.get(), nh))
      {
        PAL_THROW_DEFAULT("problem configuring cop constraint");
      }
      cop_constraints_tasks_.push_back(cop_constraint);
    }
  }

  {
    eMatrixHom initial_relative_foot_pose(createMatrix(
        eQuaternion::Identity(), eVector3(0., bp->leg_kinematic_description_.hip_spacing_, 0.)));
    // Relative goto task reference
    foot_relative_pose_target_.reset(
        new PointerReference(nh, bp->right_foot_name_ + "_relative", bp->left_foot_name_,
                             bp->left_foot_name_, initial_relative_foot_pose.translation(),
                             eQuaternion(initial_relative_foot_pose.rotation())));
  }

  {
    wbc_gains_->relative_foot_position_xy_gains_ =
        boost::make_shared<pal_ros_utils::PIDRateLimitedParameters>(
            "relative_foot_position_xy_gains", nh);
    wbc_gains_->relative_foot_position_xy_gains_->p_gain_ = parameters_->foot_relative_stance_xy_gain_;

    // Relative goto task XY
    GoToRelativeDynamicParameters go_to_relative_foot_pose_xy_parameters;
    go_to_relative_foot_pose_xy_parameters.tip_name_ = bp->right_foot_name_;
    go_to_relative_foot_pose_xy_parameters.tip_respect_name_ = bp->left_foot_name_;
    go_to_relative_foot_pose_xy_parameters.reference = foot_relative_pose_target_;
    go_to_relative_foot_pose_xy_parameters.coordinates = { coord_t::X, coord_t::Y };
    go_to_relative_foot_pose_xy_parameters.setPositionParams(
        wbc_gains_->relative_foot_position_xy_gains_);

    for (size_t i = 0; i < go_to_relative_foot_pose_xy_parameters.coordinates.size(); ++i)
    {
      go_to_relative_foot_pose_xy_parameters.lower_bound.push_back(0.0);
      go_to_relative_foot_pose_xy_parameters.upper_bound.push_back(0.0);
      go_to_relative_foot_pose_xy_parameters.bound_type.push_back(Bound::BOUND_TWIN);
    }

    foot_relative_pose_xy_task_.reset(new GoToRelativeDynamicTask());
    if (!foot_relative_pose_xy_task_->setUp("foot_relative_xy", go_to_relative_foot_pose_xy_parameters,
                                            dynamic_stack_.get(), nh))
    {
      PAL_THROW_DEFAULT("problem configuring foot relative task xy");
    }

    foot_relative_pose_xy_task_->setWeight(100.);
  }

  {
    wbc_gains_->relative_foot_orientation_yaw_gains_ =
        boost::make_shared<pal_ros_utils::PIDRateLimitedParameters>(
            "relative_foot_position_yaw_gains", nh);

    wbc_gains_->relative_foot_orientation_yaw_gains_->p_gain_ = parameters_->foot_relative_stance_yaw_gain_;
    // Relatice goto task Yaw
    GoToRelativeDynamicParameters go_to_relative_foot_pose_yaw_parameters;
    go_to_relative_foot_pose_yaw_parameters.tip_name_ = bp->right_foot_name_;
    go_to_relative_foot_pose_yaw_parameters.tip_respect_name_ = bp->left_foot_name_;
    go_to_relative_foot_pose_yaw_parameters.reference = foot_relative_pose_target_;
    go_to_relative_foot_pose_yaw_parameters.coordinates = { coord_t::Yaw };
    go_to_relative_foot_pose_yaw_parameters.setOrientationParams(
        wbc_gains_->relative_foot_orientation_yaw_gains_);

    for (size_t i = 0; i < go_to_relative_foot_pose_yaw_parameters.coordinates.size(); ++i)
    {
      go_to_relative_foot_pose_yaw_parameters.lower_bound.push_back(0.0);
      go_to_relative_foot_pose_yaw_parameters.upper_bound.push_back(0.0);
      go_to_relative_foot_pose_yaw_parameters.bound_type.push_back(Bound::BOUND_TWIN);
    }

    foot_relative_pose_yaw_task_.reset(new GoToRelativeDynamicTask());
    if (!foot_relative_pose_yaw_task_->setUp("foot_relative_yaw",
                                             go_to_relative_foot_pose_yaw_parameters,
                                             dynamic_stack_.get(), nh))
    {
      PAL_THROW_DEFAULT("problem configuring foot relative task yaw");
    }

    foot_relative_pose_yaw_task_->setWeight(100.);
  }

  /*
  // COP Tracking
  if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced))
  {
    for (size_t i = 0; i < 2; ++i)
    {
      VectorReferenceAbstractPtr reference(new VectorPointerReference());
      reference->configure(nh, 2, "cop_reference_" + foot_names_[i]);
      reference->setPositionTarget(eVector2::Zero());
      cop_references_.push_back(reference);

      COPTracking6dParam dynamic_params;
      dynamic_params.footFrame = foot_names_[i];
      dynamic_params.reference = reference;

      COPTracking6dDynamicPtr cop_tracking(new COPTracking6dDynamic());
      if (!cop_tracking->setUp("cop_tracking_" + foot_names_[i], dynamic_params,
                               dynamic_stack_.get(), nh))
      {
        PAL_THROW_DEFAULT("problem configuring cop constraint");
      }
      cop_tracking->setWeight(parameters_->cop_tracking_weight_);
      cop_tracking_tasks_.push_back(cop_tracking);
    }
  }
  */
  if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced))
  {
    for (size_t i = 0; i < foot_names_.size(); ++i)
    {
      FrictionConstratintDynamicMetaTaskPtr friction_constraint(new FrictionConstratintDynamicMetaTask(
          "friction_cone_constraint_" + foot_names_[i], dynamic_stack_.get(),
          parameters_->friction_mu_, foot_names_[i], nh));
      friction_constratin_tasks_.push_back(friction_constraint);
    }
  }

  if (zeroComXYGain_)
  {
    com_xy_task_.reset(new GoToXYCOMDynamicMetaTask("com_xy", dynamic_stack_.get(),
                                                    base_target_, nh, 0, 0, false));
  }
  else
  {
    com_xy_task_.reset(new GoToXYCOMDynamicMetaTask(
        "com_xy", dynamic_stack_.get(), base_target_, nh, parameters_->com_xy_p_gain_,
        parameters_->com_xy_d_gain_, parameters_->com_xy_critically_damped_));
  }
  com_xy_task_->setWeight(parameters_->com_xy_task_weight_);
  wbc_gains_->com_xy_gains_ = com_xy_task_->getParameters()->pid_parameters_;

  com_z_task_.reset(new GoToZCOMDynamicMetaTask(
      "com_z", dynamic_stack_.get(), base_target_, nh, parameters_->com_z_p_gain_,
      parameters_->com_z_d_gain_, parameters_->com_z_critically_damped_));
  com_z_task_->setWeight(parameters_->com_z_task_weight_);
  wbc_gains_->com_z_gains_ = com_z_task_->getParameters()->pid_parameters_;

  orientation_com_task_.reset(new GoToOrientationDynamicMetaTask(
      "orientation_base", dynamic_stack_.get(), "base_link", base_target_, nh,
      parameters_->orientation_base_link_p_gain_, parameters_->orientation_base_link_d_gain_,
      parameters_->orientation_base_link_critically_damped_));
  orientation_com_task_->setWeight(parameters_->orientation_base_link_weight_);
  wbc_gains_->com_orientation_gains_ =
      orientation_com_task_->getParameters()->getOrientationParams();

  // Carefull orientation torso 2 link is not the same as baselink
  orientation_upper_torso_task_.reset(new GoToOrientationDynamicMetaTask(
      "orientation_torso", dynamic_stack_.get(), "torso_2_link", torso_target_, nh,
      parameters_->orientation_upper_torso_p_gain_, parameters_->orientation_upper_torso_d_gain_,
      parameters_->orientation_upper_torso_critically_damped_));
  orientation_upper_torso_task_->setWeight(parameters_->orientation_upper_torso_weight_);
  wbc_gains_->torso_orientation_gains_ =
      orientation_upper_torso_task_->getParameters()->getOrientationParams();

  // Double support force distribution
  {
    ds_force_distribution_.reset(new ForceDistributionDsDynamicTaskMetaTask(
        "ds_force_distribution", dynamic_stack_.get(), "left_sole_link",
        "right_sole_link", 0.5, nh));
    ds_force_distribution_->setWeight(parameters_->ds_force_distribution_weight_);
  }

  /*
  ds_normal_distribution_.reset(new NormalTorqueDistributionDsDynamicTaskMetaTask(
      "ds_normal_torque_districution", dynamic_stack_.get(), "left_sole_link",
      "right_sole_link", 0.5, nh));
  ds_normal_distribution_->setWeight(parameters_->ds_normal_distribution_weight_);
  */

  {
    ForceRegularizationParams force_params;
    force_regularization_.reset(new ForceRegularizationDynamicTask());
    force_regularization_->setUp("force_regularization", force_params, dynamic_stack_.get(), nh);
    force_regularization_->setWeight(parameters_->force_regularization_weight_);
  }

  {
    if (dynamic_stack_->getNumberContactTorques() > 0)
    {
      TorqueRegularizationParams torque_params;
      torque_regularization_.reset(new TorqueRegularizationDynamicTask());
      torque_regularization_->setUp("torque_regularization", torque_params,
                                    dynamic_stack_.get(), nh);
      torque_regularization_->setWeight(parameters_->torque_regularization_weight_);
    }
  }
  //  joint_torque_regularization_.reset(new TorqueDampingDynamicTaskAllJointsMetaTask(
  //      "joint_torque_regularization", dynamic_stack_.get(),
  //      dynamic_stack_->getJointNames(), nh));
  //  joint_torque_regularization_->setWeight(parameters_->joint_torque_regularization_weight_);

  wbc_gains_->swing_foot_position_xy_gains_.reset(
      new pal_ros_utils::PIDRateLimitedParameters("swing_leg_position_xy_gains", nh));
  wbc_gains_->swing_foot_position_xy_gains_->p_gain_ = parameters_->swing_leg_position_xy_p_gain_;
  wbc_gains_->swing_foot_position_z_gains_.reset(new pal_ros_utils::PIDRateLimitedParameters());
  wbc_gains_->swing_foot_position_z_gains_->p_gain_ = parameters_->swing_leg_position_z_p_gain_;
  wbc_gains_->swing_foot_orientation_roll_pitch_gains_.reset(
      new pal_ros_utils::PIDRateLimitedParameters("swing_leg_orientation_roll_pitch_gains", nh));
  wbc_gains_->swing_foot_orientation_roll_pitch_gains_->p_gain_ =
      parameters_->swing_leg_orientation_roll_pitch_p_gain_;
  wbc_gains_->swing_foot_orientation_yaw_gains_.reset(
      new pal_ros_utils::PIDRateLimitedParameters("swing_leg_orientation_yaw_gains", nh));
  wbc_gains_->swing_foot_orientation_yaw_gains_->p_gain_ =
      parameters_->swing_leg_orientation_yaw_p_gain_;

  for (size_t i = 0; i < foot_names_.size(); ++i)
  {
    GoToPositionDynamicMetaTaskPtr swing_leg_poistion_xy_task(new GoToPositionDynamicMetaTask(
        "swing_leg_position_xy_" + foot_names_[i], dynamic_stack_.get(), foot_names_[i],
        eVector3(0., 0., 0.), swing_leg_targets_[i], nh,
        wbc_gains_->swing_foot_position_xy_gains_, { coord_t::X, coord_t::Y }));
    swing_leg_poistion_xy_task->setWeight(parameters_->swing_leg_position_weight_);
    swing_leg_position_xy_tasks_.push_back(swing_leg_poistion_xy_task);

    GoToPositionDynamicMetaTaskPtr swing_leg_poistion_z_task(new GoToPositionDynamicMetaTask(
        "swing_leg_position_z_" + foot_names_[i], dynamic_stack_.get(), foot_names_[i],
        eVector3(0., 0., 0.), swing_leg_targets_[i], nh,
        wbc_gains_->swing_foot_position_z_gains_, { coord_t::Z }));
    swing_leg_poistion_z_task->setWeight(parameters_->swing_leg_position_weight_);
    swing_leg_position_z_tasks_.push_back(swing_leg_poistion_z_task);

    GoToOrientationDynamicMetaTaskPtr swing_leg_orientation_roll_pitch_task(
        new GoToOrientationDynamicMetaTask(
            "swing_leg_orientation_roll_pitch" + foot_names_[i], dynamic_stack_.get(),
            foot_names_[i], nh, swing_leg_targets_[i],
            wbc_gains_->swing_foot_orientation_roll_pitch_gains_,
            { coord_t::Roll, coord_t::Pitch }));
    swing_leg_orientation_roll_pitch_task->setWeight(parameters_->swing_leg_orientation_weight_);
    swing_leg_orientation_roll_pitch_tasks_.push_back(swing_leg_orientation_roll_pitch_task);

    GoToOrientationDynamicMetaTaskPtr swing_leg_orientation_yaw_task(new GoToOrientationDynamicMetaTask(
        "swing_leg_orientation_yaw" + foot_names_[i], dynamic_stack_.get(),
        foot_names_[i], nh, swing_leg_targets_[i],
        wbc_gains_->swing_foot_orientation_yaw_gains_, { coord_t::Yaw }));
    swing_leg_orientation_yaw_task->setWeight(parameters_->swing_leg_orientation_weight_);
    swing_leg_orientation_yaw_tasks_.push_back(swing_leg_orientation_yaw_task);
  }

  //  //  Angular Momentum
  angular_momentum_reference_.reset(
      new pal_robot_tools::VectorPointerReference(nh, 3, "angular_momentum_reference"));
  angular_momentum_task_.reset(new AngularMomentumTargetDynamicTask(
      "angular_momentum", dynamic_stack_.get(), nh, angular_momentum_reference_,
      parameters_->angular_momentum_gain_));
  angular_momentum_task_->setWeight(parameters_->angular_momentum_weight_);

  // Generic metatasks
  constraint_metatasks_.reset(new GenericMetaTask(nh, dynamic_stack_.get(), {}, "constraints"));
  objective_metatasks_.reset(new GenericMetaTask(nh, dynamic_stack_.get(), {}, "objectives"));
}

void WalkingWBCTalosBase::updateTasks(
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
    const std::vector<Eigen::Vector3d> &ee_angular_acceleration,
    task_container_vector &constraint_tasks, task_container_vector &objective_tasks)
{
  if (com_offset_helper_)
  {
    com_offset_helper_->applyOffsetsModel();
  }

  // Modify contact parameters
  actual_contact_force_description_.clear();
  actual_contact_torque_description_.clear();

  std::vector<TaskAbstractPtr> constraint_position_tasks;
  std::vector<TaskAbstractPtr> constraint_orientation_tasks;
  for (size_t i = 0; i < stance_leg_ids.size(); ++i)
  {
    {
      std::vector<unsigned int> &position_contact_ids =
          stance_leg_position_contact_description_ids_[static_cast<int>(stance_leg_ids[i])];
      for (size_t j = 0; j < position_contact_ids.size(); ++j)
      {
        actual_contact_force_description_.push_back(
            contact_forces_description_[position_contact_ids[j]]);
      }

      constraint_position_tasks.push_back(
          position_constraint_xy_tasks_[static_cast<int>(stance_leg_ids[i])]);
      constraint_position_tasks.push_back(
          position_constraint_z_tasks_[static_cast<int>(stance_leg_ids[i])]);
    }

    {
      std::vector<unsigned int> &orientation_contact_ids =
          stance_leg_orientation_contact_descrption_ids_[static_cast<int>(stance_leg_ids[i])];
      for (size_t j = 0; j < orientation_contact_ids.size(); ++j)
      {
        actual_contact_torque_description_.push_back(
            contact_torques_description_[orientation_contact_ids[j]]);
      }
      constraint_orientation_tasks.push_back(
          orientation_constraint_roll_pitch_tasks_[static_cast<int>(stance_leg_ids[i])]);
      constraint_orientation_tasks.push_back(
          orientation_constraint_yaw_tasks_[static_cast<int>(stance_leg_ids[i])]);
    }
  }

  dynamic_stack_->updateContactConfiguration(actual_contact_force_description_,
                                             actual_contact_torque_description_, solution_);


  if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced))
  {
    //    setUpPhysics(dynamic_stack_, unilateral_forces_task_, dynamic_task_,
    //                 constraint_position_tasks, constraint_orientation_tasks,
    //                 constraint_tasks);
    /// @todo hack to overcome the infeasability of equality constraints
    setUpPhysics(dynamic_stack_, unilateral_forces_task_, dynamic_task_, {}, {}, constraint_tasks);

    for (size_t i = 0; i < constraint_position_tasks.size(); ++i)
    {
      constraint_position_tasks[i]->setWeight(1);
      constraint_tasks.push_back(constraint_position_tasks[i]);
    }

    for (size_t i = 0; i < constraint_orientation_tasks.size(); ++i)
    {
      constraint_orientation_tasks[i]->setWeight(1);
      constraint_tasks.push_back(constraint_orientation_tasks[i]);
    }
  }
  else if (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced_friction_cone)
  {
    //    /// @todo hack to overcome the infeasability of equality constraints
    //    setUpPhysics(dynamic_stack_, unilateral_forces_task_, dynamic_task_, {}, {},
    //    constraint_tasks);

    for (size_t i = 0; i < constraint_position_tasks.size(); ++i)
    {
      constraint_position_tasks[i]->setWeight(100);
      objective_tasks.push_back(constraint_position_tasks[i]);
    }

    for (size_t i = 0; i < constraint_orientation_tasks.size(); ++i)
    {
      constraint_orientation_tasks[i]->setWeight(100);
      objective_tasks.push_back(constraint_orientation_tasks[i]);
    }

    constraint_tasks.push_back(dynamic_task_);
    constraint_tasks.push_back(unilateral_forces_task_);
    //    setUpPhysics(dynamic_stack_, unilateral_forces_task_, dynamic_task_,
    //                 constraint_position_tasks, constraint_orientation_tasks,
    //                 constraint_tasks);
  }
  else
  {
    PAL_THROW_DEFAULT("formulation not supported");
  }


  if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced))
  {
    // Friction constraints
    for (size_t i = 0; i < stance_leg_ids.size(); ++i)
    {
      constraint_tasks.push_back(friction_constratin_tasks_[static_cast<int>(stance_leg_ids[i])]);
    }
  }


  if (parameters_->cop_margin_x_ > 0.0 || parameters_->cop_margin_y_ > 0.0)
  {
    if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
        (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced) ||
        (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced_friction_cone))
    {
      // COP Constraint tasks
      for (size_t i = 0; i < stance_leg_ids.size(); ++i)
      {
        constraint_tasks.push_back(cop_constraints_tasks_[static_cast<int>(stance_leg_ids[i])]);
      }
    }
  }

  /*
  if ((dynamic_stack_->getFormulationType() == +formulation_t::dynamics) ||
      (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced))
  {
    //  // COP tracking tasks
    for (size_t i = 0; i < stance_leg_ids.size(); ++i)
    {
      objective_tasks.push_back(cop_tracking_tasks_[stance_leg_ids[i]]);
    }
  }
  */
  //// Objectives

  if (parameters_->angular_momentum_weight_ > 0.0)
  {
    angular_momentum_reference_->setPositionTarget(desiredAngularMomentum);
    angular_momentum_reference_->setVelocityTarget(desiredAngularMomentumVelocity);
    objective_tasks.push_back(angular_momentum_task_);
  }

  // Force weight distribution, only used if the both foot are stance
  if (stance_leg_ids.size() == 2)
  {
    if (parameters_->ds_force_distribution_weight_ > 0.0)
    {
      ds_force_distribution_->setDistributionWeight(weightDistribution);
      objective_tasks.push_back(ds_force_distribution_);
    }

    /*
    /// @todo warning this was wrong until now
    ds_normal_distribution_->setDistributionWeight(weightDistribution);
    objective_tasks.push_back(ds_normal_distribution_);
    */

    eMatrixHom left_foot_stance_pose =
        createMatrix(stance_leg_orientations[0], stance_leg_positions[0]);
    eMatrixHom right_foot_stance_pose =
        createMatrix(stance_leg_orientations[1], stance_leg_positions[1]);
    eMatrixHom local_foot_target = left_foot_stance_pose.inverse() * right_foot_stance_pose;

    foot_relative_pose_target_->setPoseTarget(local_foot_target);

    //objective_tasks.push_back(foot_relative_pose_xy_task_);
    //objective_tasks.push_back(foot_relative_pose_yaw_task_);
  }

  base_target_->setPositionTarget(comPosition);
  base_target_->setLinearVelocityTarget(comVelocity);
  base_target_->setLinearAccelerationTarget(comAcceleration);

  base_target_->setOrientationTarget(baseOrientation);
  base_target_->setAngularVelocityTarget(baseAngularVelocity);
  base_target_->setAngularAccelerationTarget(baseAngularAcceleration);

  torso_target_->setOrientationTarget(torsoOrientation);
  torso_target_->setAngularVelocityTarget(torsoAngularVelocity);
  torso_target_->setAngularAccelerationTarget(torsoAngularAcceleration);

  objective_tasks.push_back(com_xy_task_);
  objective_tasks.push_back(com_z_task_);
  objective_tasks.push_back(orientation_com_task_);

  // Stance leg tasks
  for (size_t i = 0; i < stance_leg_ids.size(); ++i)
  {
    ReferenceAbstractPtr foot_target =
        stance_leg_targets_[static_cast<int>(stance_leg_ids[i])];
    foot_target->setPositionTarget(stance_leg_positions[i]);
    foot_target->setOrientationTarget(stance_leg_orientations[i]);
  }


  // Swing leg tasks
  for (size_t i = 0; i < swing_leg_ids.size(); ++i)
  {
    ReferenceAbstractPtr foot_target = swing_leg_targets_[static_cast<int>(swing_leg_ids[i])];

    foot_target->setPositionTarget(swingLegPositions[i]);
    foot_target->setOrientationTarget(swingLegOrientations[i]);
    foot_target->setVelocityTarget(swingLegLinearVelocitys[i], swingLegAngularVelocitys[i]);
    foot_target->setLinearAccelerationTarget(swingLegLinearAcceleartions[i]);
    foot_target->setAngularAccelerationTarget(swingLegAngularAcceleartions[i]);

    objective_tasks.push_back(swing_leg_position_xy_tasks_[static_cast<int>(swing_leg_ids[i])]);
    objective_tasks.push_back(swing_leg_position_z_tasks_[static_cast<int>(swing_leg_ids[i])]);
    objective_tasks.push_back(
        swing_leg_orientation_roll_pitch_tasks_[static_cast<int>(swing_leg_ids[i])]);
    objective_tasks.push_back(
        swing_leg_orientation_yaw_tasks_[static_cast<int>(swing_leg_ids[i])]);
  }

  if (parameters_->force_regularization_weight_ > 0.0)
  {
    objective_tasks.push_back(force_regularization_);
  }

  if (parameters_->torque_regularization_weight_ > 0.0)
  {
    if (dynamic_stack_->getNumberContactTorques() > 0)
    {
      objective_tasks.push_back(torque_regularization_);
    }
  }
  //  objective_tasks.push_back(joint_torque_regularization_);
}


void WalkingWBCTalosBase::processSolution(
    const std::vector<Side> stanceLegIDs, const Eigen::VectorXd &Q, const Eigen::VectorXd &QDot,
    Eigen::VectorXd &Tau, Eigen::Vector3d &floatingBaseDesiredLinearAcceleration,
    Eigen::Vector3d &floatingBaseDesiredAngularAcceleration,
    Eigen::VectorXd &jointStateDesiredAcceleration,
    std::vector<std::pair<Eigen::Vector3d, pal_wbc::ContactDescription>> &computed_forces,
    std::vector<std::pair<Eigen::Vector3d, pal_wbc::ContactDescription>> &computed_torques)
{
  computed_forces.resize(dynamic_stack_->getNumberContactForces());
  computed_torques.resize(dynamic_stack_->getNumberContactTorques());

  if (dynamic_stack_->getFormulationType() == +formulation_t::dynamics)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      floatingBaseDesiredLinearAcceleration(i) =
          solution_(i + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() - 6);
    }
    for (size_t i = 3; i < 6; ++i)
    {
      floatingBaseDesiredAngularAcceleration(i - 3) =
          solution_(i + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() - 6);
    }
    for (size_t i = 6; i < dynamic_stack_->getNumberDofJointState() + 6; ++i)
    {
      jointStateDesiredAcceleration(i - 6) =
          solution_(i + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() - 6);
    }

    for (size_t i = 0; i < dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() - 6; ++i)
    {
      Tau(i) = solution_(i);
    }

    PAL_ASSERT_PERSIST_EQUAL(computed_forces.size(), actual_contact_force_description_.size());
    for (size_t i = 0; i < computed_forces.size(); ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        computed_forces[i].first(j) = solution_(
            j + i * 3 + 2 * dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() - 6);
      }
      computed_forces[i].second = actual_contact_force_description_[i];
    }

    for (size_t i = 0; i < computed_torques.size(); ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        computed_torques[i].first(j) = solution_(
            j + i * 3 + 2 * dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() -
            6 + 3 * computed_forces.size());
      }
      computed_torques[i].second = actual_contact_torque_description_[i];
    }

    createGeneralizedAccelerationVector(
        floatingBaseDesiredLinearAcceleration, floatingBaseDesiredAngularAcceleration,
        jointStateDesiredAcceleration, generalized_acceleration_);
  }
  else if (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      floatingBaseDesiredLinearAcceleration(i) = solution_(i);
    }
    for (size_t i = 3; i < 6; ++i)
    {
      floatingBaseDesiredAngularAcceleration(i - 3) = solution_(i);
    }
    for (size_t i = 6; i < dynamic_stack_->getNumberDofJointState() + 6; ++i)
    {
      jointStateDesiredAcceleration(i - 6) = solution_(i);
    }

    // Compute the torques using inverse dynamics
    computed_forces.resize(dynamic_stack_->getNumberContactForces());
    computed_torques.resize(dynamic_stack_->getNumberContactTorques());
    for (size_t i = 0; i < computed_forces.size(); ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        computed_forces[i].first(j) = solution_(
            j + i * 3 + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase());
      }
      computed_forces[i].second = actual_contact_force_description_[i];
    }
    for (size_t i = 0; i < computed_torques.size(); ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        computed_torques[i].first(j) = solution_(
            j + i * 3 + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() +
            3 * computed_forces.size());
      }
      computed_torques[i].second = actual_contact_torque_description_[i];
    }

    createGeneralizedAccelerationVector(
        floatingBaseDesiredLinearAcceleration, floatingBaseDesiredAngularAcceleration,
        jointStateDesiredAcceleration, generalized_acceleration_);

    dynamic_stack_->computeInverseDynamicsMomentum(Q, QDot, generalized_acceleration_,
                                                   computed_forces, computed_torques, Tau);
  }
  else if (dynamic_stack_->getFormulationType() == +formulation_t::dynamics_reduced_friction_cone)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      floatingBaseDesiredLinearAcceleration(i) = solution_(i);
    }
    for (size_t i = 3; i < 6; ++i)
    {
      floatingBaseDesiredAngularAcceleration(i - 3) = solution_(i);
    }
    for (size_t i = 6; i < dynamic_stack_->getNumberDofJointState() + 6; ++i)
    {
      jointStateDesiredAcceleration(i - 6) = solution_(i);
    }

    for (size_t i = 0; i < computed_forces.size(); ++i)
    {
      Eigen::VectorXd basis_force(4);
      for (size_t j = 0; j < 4; ++j)
      {
        basis_force(j) = solution_(
            j + i * 4 + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase());
      }

      computed_forces[i].first = friction_cone_basis_ * basis_force;
      computed_forces[i].second = actual_contact_force_description_[i];
    }
    for (size_t i = 0; i < computed_torques.size(); ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        computed_torques[i].first(j) = solution_(
            j + i * 3 + dynamic_stack_->getNumberDofJointStateIncludingFloatingBase() +
            4 * computed_forces.size());
      }
      computed_torques[i].second = actual_contact_torque_description_[i];
    }

    createGeneralizedAccelerationVector(
        floatingBaseDesiredLinearAcceleration, floatingBaseDesiredAngularAcceleration,
        jointStateDesiredAcceleration, generalized_acceleration_);

    dynamic_stack_->computeInverseDynamicsMomentum(Q, QDot, generalized_acceleration_,
                                                   computed_forces, computed_torques, Tau);
  }
  else
  {
    PAL_THROW_DEFAULT("Formulation not supported");
  }

  //  assert(forwardDynamicsVerification(*dynamic_stack_->getModel(),
  //                                     Q,
  //                                     QDot,
  //                                     generalizedAcceleration_,
  //                                     Tau,
  //                                     contactForces,
  //                                     contactTorques,
  //                                     computedForces,
  //                                     computedTorques));

  /*
  if(dynamic_stack_->getFormulationType() == formulation_t::dynamics){
    dynamic_stack_->publishForces(time, solution_, marray_, index_);
  }
  else if(dynamic_stack_->getFormulationType() == formulation_t::dynamics_reduced){
    dynamic_stack_->publishForces(time, solution_, marray_, index_);

  }
  else if(dynamic_stack_->getFormulationType() ==
  formulation_t::dynamics_friction_cone){
    dynamic_stack_->publishFrictionConeForces(time, solution_, marray_, index_);
  }
  else{
    ROS_ERROR_STREAM("Formulation not supported");
  }
  */

  SupporType st;
  Side side;
  if (stanceLegIDs.size() == 2)
  {
    st = +SupporType::DS;
    side = +Side::LEFT;
  }
  else
  {
    st = SupporType::SS;
    if (stanceLegIDs[0] == +Side::LEFT)
    {
      side = +Side::LEFT;
    }
    else
    {
      side = Side::RIGHT;
    }
  }

  // publish_sphere(combinedCop_, RED, MEDIUM, "world", "total_cop", time, marray_,
  // index_);markers
  // publish_text(combinedCop_, "COP", RED, "world", "total_cop", time, marray_,
  // index_);

  // dynamic_stack_->visualizeCOM(time, solution_, marray_, index_);

  // Reset existing markers
  //  visualization_msgs::MarkerArray resetMarker;
  //  resetMarker.markers.resize(1);
  //  resetMarker.markers[0].header.frame_id = "world";
  //  resetMarker.markers[0].header.stamp = time;
  //  resetMarker.markers[0].action = 3;
  //  wbc_pub_.publish(resetMarker);

  //  if(wbc_pub_->trylock()){
  //    wbc_pub_->msg_ = marray_;
  //    wbc_pub_->unlockAndPublish();
  //  }
  //  marray_.markers.clear();
  //  index_ = 0;
}

std::vector<std::string> WalkingWBCTalosBase::getJointNames()
{
  return dynamic_stack_->getJointNames();
}
}
