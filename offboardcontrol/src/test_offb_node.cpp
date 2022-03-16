#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include "nav_msgs/Odometry.h"
#include <string.h>
#include <vector>
#include "ControlLibrary/ControllerConverter.h"
#include "ControlLibrary/DroneController.h"
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ControllerConverter Controller(nh, rate);
  DroneControl Drone(nh, rate);

    Drone.ArmDrone = false;;
    //Dtest.TakeoffAltitude = 2.5;
    Drone.InitiateTakeoff = false;

    while(ros::ok()){
        if (Controller.Arm){ // Arm when controller input says
            Drone.ArmDrone = true;
            ROS_INFO_STREAM("Sent arming command");
            Drone.cv.notify_one();
        }

        if (Controller.Takeoff){ // Takeoff when controller input says
            Drone.InitiateTakeoff = true;
            ROS_INFO_STREAM("Sent takeoff command");
            Drone.cv.notify_one();
        }

        if  (Controller.Land){ // Land when controller input says 
            Drone.InitiateLanding = true;
            ROS_INFO_STREAM("Sent landing command");
        }

        Drone.InputTargetPosition = Controller.Target; // Send other input from controller to drone
        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}
