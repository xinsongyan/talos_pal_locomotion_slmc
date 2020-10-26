#include <pal_test_utils/gtest_helpers.h>
#include <pal_physics_simulator/physics_simulator.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <dynamic_introspection/dynamic_introspection.h>
#include <pal_locomotion/walking_step_command.h>
#include <pal_locomotion/velocity_command_que.h>
#include <pal_locomotion_msgs/PushVelocityCommandsSrv.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <controller_manager_msgs/ListControllers.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <pal_locomotion/ggtest/test_utils.h>
#include <nav_msgs/Odometry.h>
#include <ariles/ariles_all.h>
#include <talos_pal_locomotion/test_utils.h>
#include <pal_configuration_initializer/configuration_initializer.h>
#include <pal_ros_utils/ariles_utils.h>
#include <pal_ros_utils/reference/pose/pose_reference_minjerk.h>

using namespace pal_locomotion;
using namespace pal_robot_tools;
using namespace pal_collision_tools;
using namespace pal_physics_simulator;

typedef boost::shared_ptr<PoseReferenceMinJerk> PoseReferenceMinJerkPtr;

namespace
{
struct PoseSetPoint : public ariles::ConfigurableBase
{
  PoseSetPoint()
  {
  }

  void setDefaults() override
  {
  }

#define ARILES_SECTION_ID "PoseSetPoint"
#define ARILES_CONSTRUCTOR PoseSetPoint
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(pose)                                                                    \
  ARILES_ENTRY_(duration)
#include ARILES_INITIALIZE

  pal::ArilesPose pose_;
  double duration_;
};

struct TestParameters : public ariles::ConfigurableBase
{
  TestParameters()
  {
  }
  void setDefaults() override
  {
    model_name_ = "";
    goal_tolerance_.setDefaults();
  }

#define ARILES_SECTION_ID "TestParameters"
#define ARILES_CONSTRUCTOR TestParameters
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(set_points)                                                              \
  ARILES_ENTRY_(model_name)                                                              \
  ARILES_ENTRY_(goal_tolerance)                                                          \
  ARILES_ENTRY_(final_position)                                                          \
  ARILES_ENTRY_(final_position_tolerance)

#include ARILES_INITIALIZE

  std::vector<PoseSetPoint> set_points_;
  std::string model_name_;
  trajectory_dispatcher::GoalTolerance goal_tolerance_;

  Eigen::Vector3d final_position_;
  Eigen::Vector3d final_position_tolerance_;
};
}

inline void test_thread(ros::NodeHandle &nh, const TestParameters &parameters,
                        const ros::Duration &dt, PoseReferenceMinJerkPtr ground_reference,
                        CollisionObjectBase *base_support, PhysicsSimulator *simulator,
                        bool *test_finished)
{
  /*
// reach default configuration using position controllers.
{
  std::string position_controllers_cmd =
      "roslaunch talos_controller_configuration position_controllers.launch "
      "robot:=" +
      parameters.model_name_;
  subprocess::RAIIPopen position_controllers(position_controllers_cmd,
                                             subprocess::output{ subprocess::PIPE });
  ros::Duration(5.0).sleep();

  trajectory_dispatcher::Commander commander;
  commander.sendDefaultConfiguration(nh, parameters.goal_tolerance_);
}
*/
  ros::Duration(10.0).sleep();  // let them die

  // proceed with torque control


  std::unique_ptr<subprocess::RAIIPopen> spawner(spawnControllersAndWait("biped_walking_dcm_controller"));


  ros::Time old_time = ros::Time::now();
  dt.sleep();
  ros::Time actual_time = ros::Time::now();

  unsigned int pose_index = 0;
  ros::Time pose_target_time =
      actual_time + ros::Duration(parameters.set_points_[pose_index].duration_);

  while (nh.ok())
  {
    actual_time = ros::Time::now();
    ros::Duration cycle_dt = actual_time - old_time;

    ground_reference->integrate(actual_time, cycle_dt);

    base_support->updatePose(ground_reference->getDesiredPose());

    if (actual_time >= pose_target_time)
    {
      std::cerr << "configuratoin: " << pose_index << std::endl;
      ++pose_index;
      if (pose_index >= parameters.set_points_.size())
      {
        break;
      }
      pose_target_time =
          actual_time + ros::Duration(parameters.set_points_[pose_index].duration_);
      ground_reference->setPoseTarget(parameters.set_points_[pose_index].pose_.getMatrix(),
                                      ros::Duration(parameters.set_points_[pose_index].duration_));
    }

    old_time = actual_time;
    dt.sleep();
  }

  Eigen::Vector3d base_link_position = simulator->getLinkPosition("base_link");

  for (size_t i = 0; i < 3; ++i)
  {
    EXPECT_NEAR(base_link_position[i], parameters.final_position_[i],
                parameters.final_position_tolerance_[i]);
  }

  *test_finished = true;
}


TEST(talos_experimental_walking, moving_platform)
{
  ros::NodeHandle nh;
  TestParameters parameters;
  parameters.readConfig<ariles::ros>(nh, "");
  PAL_ASSERT_PERSIST(false == parameters.model_name_.empty(), "Model name is not set.");

  ros::Duration dt(0.001);

  SimulationParameters simulator_parameters;
  simulator_parameters.mu_ = 0.6;
  simulator_parameters.joint_damping_ = 0.5;
  simulator_parameters.erp_ = 50.0;
  simulator_parameters.enable_joint_position_limits_ = true;
  simulator_parameters.enable_joint_effort_limits_ = true;
  simulator_parameters.update_rate_dt_fraction_ = 0.0;  // speedrun
  simulator_parameters.headless_ = true;

  //  eMatrixHom initial_floating_base_pose =
  //      createMatrix(eQuaternion::Identity(), eVector3(0., 0., 1.05));

  std::vector<std::pair<std::string, double> > initial_joint_configuration;
  //  if()
  //  initial_joint_configuration.push_back({ "leg_left_3_joint", -0.4 });
  //  initial_joint_configuration.push_back({ "leg_right_3_joint", -0.4 });
  //  initial_joint_configuration.push_back({ "leg_left_4_joint", 0.8 });
  //  initial_joint_configuration.push_back({ "leg_right_4_joint", 0.8 });
  //  initial_joint_configuration.push_back({ "leg_left_5_joint", -0.4 });
  //  initial_joint_configuration.push_back({ "leg_right_5_joint", -0.4 });
  //  initial_joint_configuration.push_back({ "arm_left_1_joint", 0.3 });
  //  initial_joint_configuration.push_back({ "arm_left_2_joint", 0.4 });
  //  initial_joint_configuration.push_back({ "arm_left_3_joint", -0.5 });
  //  initial_joint_configuration.push_back({ "arm_left_4_joint", -1.5 });
  //  initial_joint_configuration.push_back({ "arm_right_1_joint", -0.3 });
  //  initial_joint_configuration.push_back({ "arm_right_2_joint", -0.4 });
  //  initial_joint_configuration.push_back({ "arm_right_3_joint", 0.5 });
  //  initial_joint_configuration.push_back({ "arm_right_4_joint", -1.5 });

  CollisionObjectBase *base_support;
  std::vector<std::unique_ptr<CollisionObjectBase> > collision_objects;
  std::unique_ptr<CollisionObjectBase> ground_plane;
  eMatrixHom ground_tf = createMatrix(eVector3(0., 0., 0.), eVector3(0., 0., 0));
  createBox("bullet", "ground", 1., 1., 0.05, ground_tf, ground_plane, YELLOW);
  base_support = ground_plane.get();
  collision_objects.push_back(std::move(ground_plane));

  SolverParameters solver_parameters;
  solver_parameters.origin_ = "siconos";
  solver_parameters.solver_type_ = "lcp_lemke";
  solver_parameters.regularization_ = 1e-1;

  std::vector<pal_physics_simulator::pluginDescription> plugins_parameters;

  plugins_parameters.push_back(std::make_pair(
      "pal_physics_simulator::FloatingBasePublisherTopicPlugin",
      property_bag::PropertyBag("link_name", std::string("base_link"), "topic_name",
                                std::string("floating_base_pose_simulated"))));
  plugins_parameters.push_back(std::make_pair(
      "pal_physics_simulator::PhysicsSimulatorROSControlPlugin", property_bag::PropertyBag()));
  PhysicsSimulator simulator(nh, plugins_parameters, simulator_parameters, solver_parameters,
                             collision_objects, initial_joint_configuration);

  PoseReferenceMinJerkPtr ground_reference(new PoseReferenceMinJerk(
      nh, dt, "ground_reference", "map", "map", eMatrixHom::Identity()));

  bool test_finished = false;
  boost::thread t(test_thread, nh, parameters, dt, ground_reference, base_support,
                  &simulator, &test_finished);

  ros::Time total_test_time(0);
  for (size_t i = 0; i < parameters.set_points_.size(); ++i)
  {
    total_test_time += ros::Duration(parameters.set_points_[i].duration_);
  }

  while (!test_finished && nh.ok())
  {
    simulator.update();
    simulator.publishState();

    ros::spinOnce();
    PUBLISH_ASYNC_STATISTICS("/introspection_data");
  }
}


PAL_DEFINE_ROS_TEST_MAIN(argv[0])
