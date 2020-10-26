/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

#include <controller_manager_msgs/ListControllers.h>
#include <pal_utils/subprocess_raii.h>
#include <ros/node_handle.h>
#include <gtest/gtest.h>
inline bool isControllerActiveInList(const std::vector<controller_manager_msgs::ControllerState> &list,
                        const std::string &name)
{
  for (size_t i = 0; i < list.size(); i++)
  {
    if (list[i].name == name)
    {
      if(list[i].state == "running")
        return true;
      else
        return false;
    }
  }
  return false;
}

std::unique_ptr<subprocess::RAIIPopen> spawnControllersAndWait(const std::string &controller_name)
{
  ROS_INFO_STREAM("Test spawning controller " + controller_name);
  ros::NodeHandle nh;
  std::string controller_spawn_cmd;
  controller_spawn_cmd = "rosrun controller_manager spawner "
                       + controller_name +
                       " __name:=" + controller_name + "_spawner";
  std::unique_ptr<subprocess::RAIIPopen> controller_spawn_process(new subprocess::RAIIPopen(controller_spawn_cmd,
                                       subprocess::output{ subprocess::PIPE }));

  ros::WallTime start_time = ros::WallTime::now();
  std::string biped_controller = "biped_walking_dcm_controller";

  ros::ServiceClient controller_manager_srv =
      nh.serviceClient<controller_manager_msgs::ListControllers>(
          "controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;

  ROS_INFO_STREAM("Waiting for controller " + controller_name);

  while(ros::WallTime::now() - start_time < ros::WallDuration(20.0))
  {
    if (!controller_manager_srv.call(list_controllers))
      PAL_THROW_DEFAULT("Error calling list controllers");

    if(isControllerActiveInList(list_controllers.response.controller, controller_name))
      break;
  }
  if (!isControllerActiveInList(list_controllers.response.controller, controller_name))
  {
    PAL_THROW_DEFAULT("Timeout waiting for controller " + controller_name);

  }
  ROS_INFO_STREAM("Controller " + controller_name + " ready");

  return controller_spawn_process;
}

