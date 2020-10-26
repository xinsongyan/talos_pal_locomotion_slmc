#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>

struct LocomotionControllerInitializerParams
{
};

class LocomotionControllerInitializer
{
public:
  bool configure(ros::NodeHandle &nh)
  {
    load_controller_service_ = nh.serviceClient<controller_manager_msgs::LoadController>(
        "/controller_manager/load_controller");
    unload_controller_service_ = nh.serviceClient<controller_manager_msgs::UnloadController>(
        "/controller_manager/unload_controller");
    switch_controller_service_ = nh.serviceClient<controller_manager_msgs::SwitchController>(
        "/controller_manager/switch_controller");

    bool load_srv_valid = false;

    for (size_t i = 0; i < 5 && !load_srv_valid; ++i)
    {
      load_controller_service_.isValid();
      ros::Duration(0.1).sleep();
    }
    if (!load_srv_valid)
    {
      return false;
    }


    return true;
  }

  bool start()
  {
    {
      // Load default controllers
      controller_manager_msgs::LoadController load_srv;
      load_srv.request.name = "walking_controller";

      if (!load_controller_service_.call(load_srv))
      {
        return false;
      }
      if (!load_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem loading walking position controller");
        return false;
      }
    }

    {
      // Start default controllers
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.start_controllers.push_back("walking_controller");
      if (!switch_controller_service_.call(switch_srv))
      {
        return false;
      }
      if (!switch_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem starting walking position controller");
        return false;
      }
    }
    // Wait for default controllers to load
    ros::Duration(10.).sleep();

    // Stopp default controllers
    {
      // Start default controllers
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.stop_controllers.push_back("walking_controller");
      if (!switch_controller_service_.call(switch_srv))
      {
        return false;
      }
      if (!switch_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem stoopping walking position controller");
        return false;
      }
    }

    {
      // Unload default controllers
      controller_manager_msgs::UnloadController unload_srv;
      unload_srv.request.name = "walking_controller";

      if (!unload_controller_service_.call(unload_srv))
      {
        return false;
      }
      if (!unload_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem unloading default controllers");
        return false;
      }
    }
    // Check force torque sensors

    // Check joint torque sensors

    // Check base estimator

    /// @todo remove this
    ros::Duration(1.).sleep();

    {
      // Load locomotion controller
      controller_manager_msgs::LoadController load_srv;
      load_srv.request.name = "biped_walking_dcm_controller";

      if (!load_controller_service_.call(load_srv))
      {
        return false;
      }
      if (!load_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem loading walking torque controller");
        return false;
      }
    }

    {
      // Start locomotion controller
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.start_controllers.push_back("biped_walking_dcm_controller");
      if (!switch_controller_service_.call(switch_srv))
      {
        return false;
      }
      if (!switch_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem unloading walking torque controller");
        return false;
      }
    }


    return true;
  }

  bool stop()
  {
    {
      // Stop locomotion controller
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.stop_controllers.push_back("biped_walking_dcm_controller");
      if (!switch_controller_service_.call(switch_srv))
      {
        return false;
      }
      if (!switch_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem stopping walking torque controller");
        return false;
      }
    }
    // Unload default controllers
    {
      // Unload locomotion controller
      controller_manager_msgs::UnloadController unload_srv;
      unload_srv.request.name = "biped_walking_dcm_controller";

      if (!unload_controller_service_.call(unload_srv))
      {
        return false;
      }
      if (!unload_srv.response.ok)
      {
        ROS_ERROR_STREAM("problem unloading walking torque controller");
        return false;
      }
    }

    return true;
  }

private:
  ros::ServiceClient load_controller_service_;
  ros::ServiceClient unload_controller_service_;
  ros::ServiceClient switch_controller_service_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locomotion_controller_initializer");
  ros::NodeHandle nh;

  LocomotionControllerInitializer initializer;

  if (initializer.configure(nh))
  {
    ROS_ERROR_STREAM("problem configuring initializer");
    return 0;
  }

  if (!initializer.start())
  {
    ROS_ERROR_STREAM("problem starting locomotion controller");
    return 0;
  }

  while (nh.ok())
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  initializer.stop();
}
