
/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <nav_msgs/Odometry.h>
#include <pal_locomotion/ggtest/test_utils.h>
#include <pal_locomotion/velocity_command_que.h>
#include <pal_locomotion/walking_step_command.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <pal_test_utils/gtest_helpers.h>
#include <talos_pal_locomotion/test_utils.h>
#include <pal_configuration_initializer/configuration_initializer.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <eigen_checks/gtest.h>

typedef actionlib::SimpleActionClient<pal_locomotion_msgs::ExecFootStepsAction> WalkingClient;

namespace
{
class TestParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TestParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(goal_tolerance, trajectory_dispatcher::GoalTolerance)              \
  ARILES_TYPED_ENTRY_(ds_duration_sec, double)                                           \
  ARILES_TYPED_ENTRY_(ss_duration_sec, double)                                           \
  ARILES_TYPED_ENTRY_(test_duration_sec, double)                                         \
  ARILES_TYPED_ENTRY_(final_position_tolerance, double)                                  \
  ARILES_TYPED_ENTRY_(final_orientation_tolerance, double)                               \
  ARILES_TYPED_ENTRY_(model_name, std::string)                                           \
  ARILES_TYPED_ENTRY_(floating_base_topic, std::string)                                  \
  ARILES_TYPED_ENTRY_(joint_states_topic, std::string)                                   \
  ARILES_TYPED_ENTRY_(left_foot_link, std::string)                                       \
  ARILES_TYPED_ENTRY_(right_foot_link, std::string)
#include ARILES_INITIALIZE

public:
  TestParameters()
  {
    setDefaults();
  }

  virtual void setDefaults()
  {
    ds_duration_sec_ = 1.0;
    ss_duration_sec_ = 4.0;

    test_duration_sec_ = 40.0;

    floating_base_topic_ = "";
    joint_states_topic_ = "";

    left_foot_link_ = "";
    right_foot_link_ = "";

    model_name_ = "";
  }
};
}

namespace
{
bool updateModel(const TestParameters& parameters, ros::NodeHandle& nh,
                 pal::rbcomposite::URDFModel& model, Eigen::Isometry3d& left_foot,
                 Eigen::Isometry3d& right_foot)
{
  pal::rbcomposite::StateFloatingBase state;
  nav_msgs::OdometryConstPtr base_msg;

  base_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(parameters.floating_base_topic_,
                                                            nh, ros::Duration(50.0));
  model.waitForJointStateMsg(state, ros::Duration(50.0), parameters.joint_states_topic_);

  if (NULL != base_msg)
  {
    pal::convert(base_msg->pose.pose.position, state.base_.position_);
    pal::convert(base_msg->pose.pose.orientation, state.base_.orientation_);

    model.update(state);

    left_foot = model.getTagTransform(model.getLinkTag(parameters.left_foot_link_));
    right_foot = model.getTagTransform(model.getLinkTag(parameters.right_foot_link_));

    left_foot.translation().z() = 0.0;
    right_foot.translation().z() = 0.0;
    return true;
  }
  return false;
}
}

TEST(talos_pal_locomotion, swing_interpolator)
{
  ros::NodeHandle nh;
  TestParameters parameters;
  parameters.readConfig<ariles::ros>(nh);
  PAL_ASSERT_PERSIST(false == parameters.model_name_.empty(), "Model name is not set.");

  ros::Duration(5.0).sleep();

  // reach default configuration using position controllers.
  {
    std::string position_controllers_cmd =
        "roslaunch talos_controller_configuration position_controllers.launch "
        "robot:=" +
        parameters.model_name_;
    subprocess::RAIIPopen position_controllers(position_controllers_cmd,
                                               subprocess::output{ subprocess::PIPE });
    ros::Duration(5.0).sleep();

    pal::configuration_initializer::ConfigurationInitializer initializer;
    initializer.findAndReadConfigFile("talos");
    initializer.parameters_.goal_tolerance_ = parameters.goal_tolerance_;
    initializer.parameters_.direct_transition_only_ = true;
    initializer.parameters_.force_ =
        true;  // initial configuration in Gazebo is colliding
    initializer.execute();
  }
  ros::Duration(5.0).sleep();  // let them die

  // proceed with torque control
  std::unique_ptr<subprocess::RAIIPopen> spawner(
      spawnControllersAndWait("biped_walking_dcm_controller"));

  ros::ServiceClient action_push_srv = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  action_push_srv.waitForExistence(ros::Duration(30.0));

  pal_locomotion_msgs::PushActions action;
  pal_locomotion_msgs::ActionWithParameters action_type;
  action_type.action_type = "pal_locomotion::StaticWalkAction";
  action.request.actions.push_back(action_type);
  EXPECT_TRUE(action_push_srv.call(action));

  ros::Duration(5.0).sleep();

  pal::rbcomposite::URDFModel model;
  model.initializeFromParameterServer(RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion, nh);

  Eigen::Isometry3d left_foot;
  Eigen::Isometry3d right_foot;
  EXPECT_TRUE(updateModel(parameters, nh, model, left_foot, right_foot));

  eMatrixHom desired_left;
  desired_left.setIdentity();
  desired_left.translation() << 0.3, 0.1, 0.0;
  eMatrixHom desired_right;
  desired_right.setIdentity();
  desired_left.translation() << 0.3, 0.1, 0.0;

  std::vector<pal_locomotion::WalkingStepCommand> walk_commands;  // Walking comands
  {
    desired_right.setIdentity();
    desired_right.translation() << 0.3, -0.1, 0.0;

    std::vector<pal_locomotion::CartesianTrajectoryPoint> trajectory_right;
    eMatrixHom pose_right;
    pose_right.setIdentity();
    pose_right.translation() << 0.0, -0.1, 0.03;
    pal_locomotion::CartesianTrajectoryPoint pt1(
        pose_right, ros::Duration(parameters.ss_duration_sec_ / 5.));
    pose_right.translation() << -0.05, -0.1, 0.03;
    pal_locomotion::CartesianTrajectoryPoint pt2(
        pose_right, ros::Duration(2 * parameters.ss_duration_sec_ / 5.));
    pose_right.translation() << -0.05, -0.1, 0.16;
    pal_locomotion::CartesianTrajectoryPoint pt3(
        pose_right, ros::Duration(3 * parameters.ss_duration_sec_ / 5.));
    pose_right.translation() << 0.3, -0.1, 0.16;
    pal_locomotion::CartesianTrajectoryPoint pt4(
        pose_right, ros::Duration(4 * parameters.ss_duration_sec_ / 5.));
    trajectory_right.push_back(pt1);
    trajectory_right.push_back(pt2);
    trajectory_right.push_back(pt3);
    trajectory_right.push_back(pt4);

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        desired_right, ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), pal_locomotion::Side::RIGHT,
        trajectory_right));

    std::vector<pal_locomotion::CartesianTrajectoryPoint> trajectory_left;
    eMatrixHom pose_left;
    pose_left.setIdentity();
    pose_left.translation() << 0.0, 0.26, 0.05;
    pal_locomotion::CartesianTrajectoryPoint pt5(
        pose_left, ros::Duration(parameters.ss_duration_sec_ / 3.));
    trajectory_left.push_back(pt5);
    pose_left.translation() << 0.3, 0.26, 0.05;
    pal_locomotion::CartesianTrajectoryPoint pt6(
        pose_left, ros::Duration(2 * parameters.ss_duration_sec_ / 3.));
    trajectory_left.push_back(pt6);

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        desired_left, ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), pal_locomotion::Side::LEFT, trajectory_left));

//    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
//        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ros::Duration(0.5),
//        ros::Duration(0.5), +pal_locomotion::SupporType::DS));
  }

  WalkingClient walking_ac(nh, "/biped_walking_dcm_controller/footsteps_execution");

  EXPECT_TRUE(walking_ac.waitForServer(ros::Duration(40.0)));

  pal_locomotion_msgs::ExecFootStepsGoal new_goal;

  for (size_t i = 0; i < walk_commands.size(); i++)
  {
    pal_locomotion_msgs::VelocityCommandMsg command_msg =
        pal_locomotion::convert(walk_commands[i]);
    new_goal.commands.push_back(command_msg);
  }

  ROS_WARN_STREAM(new_goal);

  walking_ac.sendGoal(new_goal);

  ROS_INFO_STREAM("Wait until steps are finished");
  EXPECT_TRUE(walking_ac.waitForResult(ros::Duration(parameters.test_duration_sec_)));
  actionlib::SimpleClientGoalState state = walking_ac.getState();
  EXPECT_TRUE(state.isDone());
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED);
  ROS_INFO_STREAM("Action finished, checking final position");

  EXPECT_TRUE(updateModel(parameters, nh, model, left_foot, right_foot));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(desired_left.translation(), left_foot.translation(),
                                parameters.final_position_tolerance_));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(desired_right.translation(), right_foot.translation(),
                                parameters.final_position_tolerance_));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(desired_left.rotation(), left_foot.rotation(),
                                parameters.final_orientation_tolerance_));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(desired_right.rotation(), right_foot.rotation(),
                                parameters.final_orientation_tolerance_));
}

PAL_DEFINE_ROS_TEST_MAIN(argv[0])
