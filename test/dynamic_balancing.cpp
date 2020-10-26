#include <pal_test_utils/gtest_helpers.h>
#include <pal_physics_simulator/physics_simulator.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <dynamic_introspection/dynamic_introspection.h>
#include <pal_locomotion/walking_step_command.h>
#include <pal_locomotion/velocity_command_que.h>
#include <pal_locomotion_msgs/PushVelocityCommandsSrv.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <pal_locomotion/ggtest/test_utils.h>
#include <nav_msgs/Odometry.h>
#include <ariles/ariles_all.h>
#include <talos_pal_locomotion/test_utils.h>
#include <pal_configuration_initializer/configuration_initializer.h>
#include <pal_ros_utils/message_handling.h>
#include <pal_control_msgs/OperationalSpaceGoal.h>


using namespace pal_locomotion;


namespace
{
class COMGoal : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "COMGoal"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(desired_com_position, Eigen::Vector3d)                             \
  ARILES_TYPED_ENTRY_(expected_base_position, Eigen::Vector3d)                           \
  ARILES_TYPED_ENTRY_(duration_sec, double)
#include ARILES_INITIALIZE


public:
  COMGoal()
  {
    setDefaults();
  }

  virtual void setDefaults()
  {
    desired_com_position_.setZero();
    expected_base_position_.setZero();
    duration_sec_ = 10.0;
  }
};


class TestParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TestParameters"
#define ARILES_ENTRIES                                                                       \
  ARILES_TYPED_ENTRY_(position_control_goal_tolerance, trajectory_dispatcher::GoalTolerance) \
  ARILES_TYPED_ENTRY_(final_position_tolerance, Eigen::Vector3d)                             \
  ARILES_TYPED_ENTRY_(model_name, std::string)                                               \
  ARILES_TYPED_ENTRY_(com_control_topic, std::string)                                        \
  ARILES_TYPED_ENTRY_(com_control_frame, std::string)                                        \
  ARILES_TYPED_ENTRY_(goals, std::vector<COMGoal>)                                           \
  ARILES_TYPED_ENTRY_(subscriber_waiting_time_sec, std::size_t)                              \
  ARILES_TYPED_ENTRY_(floating_base_topic, std::string)
#include ARILES_INITIALIZE


public:
  TestParameters()
  {
    setDefaults();
  }

  virtual void setDefaults()
  {
    final_position_tolerance_.setZero();

    model_name_ = "";

    com_control_topic_ = "";
    com_control_frame_ = "";

    position_control_goal_tolerance_.setDefaults();
    subscriber_waiting_time_sec_ = 50;

    floating_base_topic_ = "";

    goals_.clear();
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
        "roslaunch talos_controller_configuration position_controllers.launch robot:=" +
        parameters.model_name_;
    subprocess::RAIIPopen position_controllers(position_controllers_cmd,
                                               subprocess::output{ subprocess::PIPE });
    ros::WallDuration(3.0).sleep();

    pal::configuration_initializer::ConfigurationInitializer initializer;
    initializer.findAndReadConfigFile("talos");
    initializer.parameters_.goal_tolerance_ = parameters.position_control_goal_tolerance_;
    initializer.parameters_.direct_transition_only_ = true;
    initializer.parameters_.force_ =
        true;  // initial configuration in Gazebo is colliding
    initializer.execute();
  }
  ros::WallDuration(3.0).sleep();  // let them die


  // proceed with torque control
    std::unique_ptr<subprocess::RAIIPopen> spawner(spawnControllersAndWait("biped_walking_dcm_controller"));


  std::string action_cmd;
  action_cmd = "roslaunch pal_locomotion push_action.launch "
               "action:=pal_locomotion::BalancingAction";
  subprocess::RAIIPopen action(action_cmd, subprocess::output{ subprocess::PIPE });


  ros::Publisher pub =
      nh.advertise<pal_control_msgs::OperationalSpaceGoal>(parameters.com_control_topic_, 1);

  for (std::size_t i = 0; (0 == pub.getNumSubscribers()) && nh.ok(); ++i)
  {
    ASSERT_LT(i, parameters.subscriber_waiting_time_sec_);
    ros::WallDuration(1.0).sleep();
    if (0 == i % 5)
    {
      ROS_INFO_STREAM("Waiting for subscriber, iter = " << i);
    }
  }


  for (const COMGoal& goal : parameters.goals_)
  {
    pal_control_msgs::OperationalSpaceGoal com_command;

    com_command.header.stamp = ros::Time::now();
    com_command.header.frame_id = parameters.com_control_frame_;
    com_command.duration = ros::Duration(goal.duration_sec_);
    com_command.pose.position.x = goal.desired_com_position_.x();
    com_command.pose.position.y = goal.desired_com_position_.y();
    com_command.pose.position.z = goal.desired_com_position_.z();

    pub.publish(com_command);


    ros::Duration(goal.duration_sec_ + 5.0).sleep();


    nav_msgs::OdometryConstPtr base_state_msg;
    base_state_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(
        parameters.floating_base_topic_, nh, ros::Duration(50.0));

    ASSERT_TRUE(NULL != base_state_msg);

    std::cerr << "actual base link position: [" << base_state_msg->pose.pose.position.x
              << " " << base_state_msg->pose.pose.position.y << " "
              << base_state_msg->pose.pose.position.z << "]" << std::endl;

    ASSERT_NEAR(base_state_msg->pose.pose.position.x, goal.expected_base_position_.x(),
                parameters.final_position_tolerance_.x());
    ASSERT_NEAR(base_state_msg->pose.pose.position.y, goal.expected_base_position_.y(),
                parameters.final_position_tolerance_.y());
    ASSERT_NEAR(base_state_msg->pose.pose.position.z, goal.expected_base_position_.z(),
                parameters.final_position_tolerance_.z());
  }
}


PAL_DEFINE_ROS_TEST_MAIN(argv[0])
