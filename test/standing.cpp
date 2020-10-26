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

class TestParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TestParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(goal_tolerance)                                                          \
  ARILES_ENTRY_(test_duration_sec)                                                       \
  ARILES_ENTRY_(floating_base_topic)                                                     \
  ARILES_ENTRY_(final_position)                                                          \
  ARILES_ENTRY_(final_position_tolerance)                                                \
  ARILES_ENTRY_(model_name)
#include ARILES_INITIALIZE

public:
  TestParameters()
  {
    setDefaults();
  }

  virtual void setDefaults()
  {
    model_name_ = "";
    goal_tolerance_.setDefaults();
    test_duration_sec_ = 40.0;
  }

  double test_duration_sec_;

  Eigen::Vector3d final_position_;
  Eigen::Vector3d final_position_tolerance_;

  std::string model_name_;
  std::string floating_base_topic_;
  trajectory_dispatcher::GoalTolerance goal_tolerance_;
};

TEST(talos_pal_locomotion, standing)
{
  ros::NodeHandle nh;
  TestParameters parameters;
  parameters.readConfig<ariles::ros>(nh, "/TestParameters");

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
    std::unique_ptr<subprocess::RAIIPopen> spawner(spawnControllersAndWait("biped_walking_dcm_controller"));


  ros::Duration(parameters.test_duration_sec_).sleep();

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
