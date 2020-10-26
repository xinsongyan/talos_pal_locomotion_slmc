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
#include <math_utils/eigen/rotation.h>
#include <pal_ros_utils/conversions.h>


namespace
{
class Step : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "Step"
#define ARILES_AUTO_DEFAULTS
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(left_or_right, pal_locomotion::Side)                               \
  ARILES_TYPED_ENTRY_(position, Eigen::Vector2d)                                         \
  ARILES_TYPED_ENTRY_(orientation_angle, double)
#include ARILES_INITIALIZE

public:
  Eigen::Isometry3d pose_;

public:
  Step()
  {
    setDefaults();
  }


  void finalize()
  {
    pose_.translation() << position_, 0.0;
    pose_.linear() << pal::math_utils::matrixRollPitchYaw(0.0, 0.0, orientation_angle_);
  }
};


class TestParameters : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TestParameters"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(steps, std::vector<Step>)                                          \
  ARILES_TYPED_ENTRY_(goal_tolerance, trajectory_dispatcher::GoalTolerance)              \
  ARILES_TYPED_ENTRY_(ds_duration_sec, double)                                           \
  ARILES_TYPED_ENTRY_(ss_duration_sec, double)                                           \
  ARILES_TYPED_ENTRY_(foot_step_position_adjustment, bool)                               \
  ARILES_TYPED_ENTRY_(foot_step_time_adjustment, bool)                                   \
  ARILES_TYPED_ENTRY_(test_duration_sec, double)                                         \
  ARILES_TYPED_ENTRY_(final_position_tolerance, Eigen::Vector3d)                         \
  ARILES_TYPED_ENTRY_(final_orientation_tolerance, double)                               \
  ARILES_TYPED_ENTRY_(model_name, std::string)                                           \
  ARILES_TYPED_ENTRY_(floating_base_topic, std::string)                                  \
  ARILES_TYPED_ENTRY_(joint_states_topic, std::string)                                   \
  ARILES_TYPED_ENTRY_(left_foot_link, std::string)                                       \
  ARILES_TYPED_ENTRY_(right_foot_link, std::string)
#include ARILES_INITIALIZE

public:
  pal_locomotion::COPDegree ds_cop_degree_;
  pal_locomotion::COPDegree ss_cop_degree_;

public:
  TestParameters()
  {
    setDefaults();
  }

  virtual void setDefaults()
  {
    steps_.clear();

    ds_duration_sec_ = 0.1;
    ss_duration_sec_ = 0.8;

    foot_step_position_adjustment_ = false;
    foot_step_time_adjustment_ = false;

    ds_cop_degree_ = pal_locomotion::COPDegree::Zero;
    ss_cop_degree_ = pal_locomotion::COPDegree::Zero;

    test_duration_sec_ = 40.0;

    final_position_tolerance_.setZero();
    final_orientation_tolerance_ = 0.0;

    model_name_ = "";

    floating_base_topic_ = "";
    joint_states_topic_ = "";

    left_foot_link_ = "";
    right_foot_link_ = "";

    goal_tolerance_.setDefaults();
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

TEST(talos_experimental_walking, stepping)
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
    std::unique_ptr<subprocess::RAIIPopen> spawner(spawnControllersAndWait("biped_walking_dcm_controller"));


  std::string walk_controller_cmd;
  walk_controller_cmd = "rosrun pal_locomotion_dcm_planner start_walking_continious_dcm "
                        "__name:=start_walking_continuous -a false -t false -p Cubic -s Zero";
  subprocess::RAIIPopen walk_controller(walk_controller_cmd,
                                        subprocess::output{ subprocess::PIPE });


  pal::rbcomposite::URDFModel model;
  model.initializeFromParameterServer(RigidBodyDynamics::FloatingBaseType::XYZ_Quaternion, nh);

  Eigen::Isometry3d left_foot;
  Eigen::Isometry3d right_foot;
  ASSERT_TRUE(updateModel(parameters, nh, model, left_foot, right_foot));

  std::vector<pal_locomotion::WalkingStepCommand> walk_commands;  // Walking comands
  {
    pal_locomotion::WalkingStepCommand ds_step(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ros::Duration(0.5),
        ros::Duration(0.5), +pal_locomotion::SupporType::DS);
    walk_commands.push_back(ds_step);
    walk_commands.push_back(ds_step);

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        right_foot, ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), pal_locomotion::Side::RIGHT));

    walk_commands.push_back(pal_locomotion::WalkingStepCommand(
        left_foot, ros::Duration(parameters.ds_duration_sec_),
        ros::Duration(parameters.ss_duration_sec_), pal_locomotion::Side::LEFT));


    for (const ::Step& step : parameters.steps_)
    {
      walk_commands.push_back(pal_locomotion::WalkingStepCommand(
          step.pose_, ros::Duration(parameters.ds_duration_sec_),
          ros::Duration(parameters.ss_duration_sec_), step.left_or_right_));
    }

    walk_commands.push_back(ds_step);
  }

  start_walking(walk_commands, parameters.foot_step_position_adjustment_,
                parameters.foot_step_time_adjustment_, parameters.ds_cop_degree_,
                parameters.ss_cop_degree_);

  ros::Duration(parameters.test_duration_sec_).sleep();


  ASSERT_TRUE(updateModel(parameters, nh, model, left_foot, right_foot));

  auto step_it = --(parameters.steps_.end());


  for (std::size_t i = 0; i < 2; ++i)
  {
    Eigen::Isometry3d ref_pose = step_it->pose_;
    Eigen::Isometry3d actual_pose;


    switch (step_it->left_or_right_)
    {
      case pal_locomotion::Side::LEFT:
        actual_pose = left_foot;
        break;
      case pal_locomotion::Side::RIGHT:
        actual_pose = right_foot;
        break;
      default:
        PAL_THROW("Internal error.");
    }

    ASSERT_NEAR(ref_pose.translation().x(), actual_pose.translation().x(),
                parameters.final_position_tolerance_.x());
    ASSERT_NEAR(ref_pose.translation().y(), actual_pose.translation().y(),
                parameters.final_position_tolerance_.y());
    ASSERT_NEAR(ref_pose.translation().z(), actual_pose.translation().z(),
                parameters.final_position_tolerance_.z());

    ASSERT_TRUE(pal::test_utils::isEigenMatrixNear(ref_pose.linear(), actual_pose.linear(),
                                                   parameters.final_orientation_tolerance_));

    --step_it;
  }
}

PAL_DEFINE_ROS_TEST_MAIN(argv[0])
