/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <pal_physics_simulator/physics_simulator.h>
#include <pal_test_utils/gtest_helpers.h>
#include <pal_configuration_initializer/configuration_initializer.h>
#include <pal_utils/subprocess_raii.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <controller_manager_msgs/ListControllers.h>
#include <talos_pal_locomotion/test_utils.h>

TEST(talos_pal_locomotion, dcm_monitor)
{
  ros::NodeHandle nh;
  std::string biped_controller = "biped_walking_dcm_controller";

  ros::ServiceClient controller_manager_srv =
      nh.serviceClient<controller_manager_msgs::ListControllers>(
          "controller_manager/list_controllers");
  ros::ServiceClient appy_wrench_srv =
      nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");

  if (!controller_manager_srv.waitForExistence(ros::Duration(5.0)))
    PAL_THROW_DEFAULT("Couldn't start service controller_manager/list_controllers");

  if (!appy_wrench_srv.waitForExistence(ros::Duration(5.0)))
    PAL_THROW_DEFAULT("Couldn't start service gazebo/apply_body_wrench");

  // proceed with torque control
  std::unique_ptr<subprocess::RAIIPopen> spawner(
      spawnControllersAndWait("biped_walking_dcm_controller"));

  // let some time for the robot to stabilize itself before pushing it
  ros::Duration(5).sleep();

  controller_manager_msgs::ListControllers list_controllers;
  gazebo_msgs::ApplyBodyWrench apply_wrench_msg;
  apply_wrench_msg.request.body_name = "torso_1_link";
  apply_wrench_msg.request.reference_frame = "base_link";
  apply_wrench_msg.request.reference_point.x = 0.0;
  apply_wrench_msg.request.reference_point.y = 0.0;
  apply_wrench_msg.request.reference_point.z = 0.0;
  apply_wrench_msg.request.duration = ros::Duration(1.0);
  apply_wrench_msg.request.start_time = ros::Time::now();
  apply_wrench_msg.request.wrench.force.x = 50.0;
  apply_wrench_msg.request.wrench.force.y = 0.0;
  apply_wrench_msg.request.wrench.force.z = 0.0;
  apply_wrench_msg.request.wrench.torque.x = 0.0;
  apply_wrench_msg.request.wrench.torque.y = 0.0;
  apply_wrench_msg.request.wrench.torque.z = 0.0;

  if (!appy_wrench_srv.call(apply_wrench_msg))
    PAL_THROW_DEFAULT("Error calling apply body wrench");

  ros::Duration(2.0).sleep();

  if (!controller_manager_srv.call(list_controllers))
    PAL_THROW_DEFAULT("Error calling list controllers");

  EXPECT_TRUE(isControllerActiveInList(list_controllers.response.controller, biped_controller));

  apply_wrench_msg.request.start_time = ros::Time::now();
  apply_wrench_msg.request.wrench.force.x = 70.0;

  if (!appy_wrench_srv.call(apply_wrench_msg))
    PAL_THROW_DEFAULT("Error calling apply body wrench");

  ros::Duration(2.0).sleep();

  if (!controller_manager_srv.call(list_controllers))
    PAL_THROW_DEFAULT("Error calling list controllers");

  ROS_INFO("Finishing DCM monitor test");
  EXPECT_FALSE(isControllerActiveInList(list_controllers.response.controller, biped_controller));
}

PAL_DEFINE_ROS_TEST_MAIN(argv[0])
