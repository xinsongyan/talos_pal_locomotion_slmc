#include <ariles/ariles_all.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <dynamic_introspection/dynamic_introspection.h>
#include <nav_msgs/Odometry.h>
#include <pal_locomotion/ggtest/test_utils.h>
#include <pal_locomotion/velocity_command_que.h>
#include <pal_locomotion/walking_step_command.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <pal_locomotion_msgs/PushVelocityCommandsSrv.h>
#include <pal_physics_simulator/physics_simulator.h>
#include <pal_test_utils/gtest_helpers.h>
#include <talos_pal_locomotion/test_utils.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <pal_configuration_initializer/configuration_initializer.h>

using namespace pal_locomotion;

namespace
{
class TestParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TestParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(goal_tolerance, trajectory_dispatcher::GoalTolerance)              \
  ARILES_TYPED_ENTRY_(ds_duration_sec, double)                                           \
  ARILES_TYPED_ENTRY_(ss_duration_sec, double)                                           \
  ARILES_TYPED_ENTRY_(linear_velocity, Eigen::Vector3d)                                  \
  ARILES_TYPED_ENTRY_(angular_velocity, Eigen::Vector3d)                                 \
  ARILES_TYPED_ENTRY_(foot_step_position_adjustment, bool)                               \
  ARILES_TYPED_ENTRY_(foot_step_time_adjustment, bool)                                   \
  ARILES_TYPED_ENTRY_(test_duration_sec, double)                                         \
  ARILES_TYPED_ENTRY_(final_position, Eigen::Vector3d)                                   \
  ARILES_TYPED_ENTRY_(final_position_tolerance, Eigen::Vector3d)                         \
  ARILES_TYPED_ENTRY_(model_name, std::string)                                           \
  ARILES_TYPED_ENTRY_(floating_base_topic, std::string)
#include ARILES_INITIALIZE

public:
  COPDegree ds_cop_degree_;
  COPDegree ss_cop_degree_;

public:
  TestParameters()
  {
    setDefaults();
  }

  virtual void setDefaults()
  {
    ds_duration_sec_ = 0.1;
    ss_duration_sec_ = 0.8;

    linear_velocity_.setZero();
    angular_velocity_.setZero();

    foot_step_position_adjustment_ = false;
    foot_step_time_adjustment_ = false;

    ds_cop_degree_ = COPDegree::Zero;
    ss_cop_degree_ = COPDegree::Zero;

    test_duration_sec_ = 40.0;

    final_position_.setZero();
    final_position_tolerance_.setZero();

    model_name_ = "";

    floating_base_topic_ = "";

    goal_tolerance_.setDefaults();
  }
};
}

TEST(talos_experimental_walking, simple_walk)
{
  ros::NodeHandle nh;
  TestParameters parameters;
  parameters.readConfig<ariles::ros>(nh);
  PAL_ASSERT_PERSIST(false == parameters.model_name_.empty(), "Model name is not set.");

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


  std::string action_cmd;
  action_cmd = "roslaunch pal_locomotion push_action.launch "
               "action:=pal_locomotion::StaticWalkAction";
  subprocess::RAIIPopen action(action_cmd, subprocess::output{ subprocess::PIPE });
  ROS_INFO_STREAM("Test waiting until static walk action is pushed");
  action.wait();
  ROS_INFO_STREAM("Done waiting until static walk action is pushed");


  std::vector<pal_locomotion::WalkingStepCommand> walk_commands;  // Walking comands
  {
    pal_locomotion::WalkingStepCommand ds_step(Eigen::Vector3d::Zero(),
                                               Eigen::Vector3d::Zero(), ros::Duration(0.5),
                                               ros::Duration(0.5), +SupporType::DS);
    walk_commands.push_back(ds_step);
    walk_commands.push_back(ds_step);

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        parameters.linear_velocity_, parameters.angular_velocity_,
        ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), +SupporType::SS, +Side::LEFT));

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        parameters.linear_velocity_, parameters.angular_velocity_,
        ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), +SupporType::SS, +Side::RIGHT));

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        parameters.linear_velocity_, parameters.angular_velocity_,
        ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), +SupporType::SS, +Side::LEFT));

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        parameters.linear_velocity_, parameters.angular_velocity_,
        ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), +SupporType::SS, +Side::RIGHT));

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        parameters.linear_velocity_, parameters.angular_velocity_,
        ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), +SupporType::DS));
  }

  actionlib::SimpleActionClient<pal_locomotion_msgs::ExecFootStepsAction> walking_ac(
      nh, "/biped_walking_dcm_controller/footsteps_execution");
  ASSERT_TRUE(walking_ac.waitForServer(ros::Duration(10.0)));

  pal_locomotion_msgs::ExecFootStepsGoal goal;
  for (size_t i = 0; i < walk_commands.size(); i++)
  {
    // Convert from WalkingStepCommand -> VelocityCommandMsg
    pal_locomotion_msgs::VelocityCommandMsg command_msg =
        pal_locomotion::convert(walk_commands[i]);
    goal.commands.push_back(command_msg);
  }

  walking_ac.sendGoal(goal);

  ASSERT_TRUE(walking_ac.waitForResult(ros::Duration(parameters.test_duration_sec_)));

  ros::Duration(5.0).sleep();

  nav_msgs::OdometryConstPtr base_state_msg;
  base_state_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(
      parameters.floating_base_topic_, nh, ros::Duration(50.0));

  ASSERT_TRUE(NULL != base_state_msg);

  ROS_ERROR_STREAM("actual base link position: ["
                   << base_state_msg->pose.pose.position.x << " "
                   << base_state_msg->pose.pose.position.y << " "
                   << base_state_msg->pose.pose.position.z << "]");

  ASSERT_NEAR(base_state_msg->pose.pose.position.x, parameters.final_position_.x(),
              parameters.final_position_tolerance_.x());
  ASSERT_NEAR(base_state_msg->pose.pose.position.y, parameters.final_position_.y(),
              parameters.final_position_tolerance_.y());
  ASSERT_NEAR(base_state_msg->pose.pose.position.z, parameters.final_position_.z(),
              parameters.final_position_tolerance_.z());
}

PAL_DEFINE_ROS_TEST_MAIN(argv[0])
