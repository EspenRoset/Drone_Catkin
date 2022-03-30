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
#include <std_msgs/Float32MultiArray.h>


ros::Publisher test;
std_msgs::Float32MultiArray testArray;
std::thread changeVectorThread;
std::vector<float> goVector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> stopVector = {1.0, 100.0, 100.0, -2.0, 0.0};



void WaitThenChangeVector(){ // Can be used for testing avoidance system
  while(true){
  testArray.data = goVector;
  
  ROS_INFO("Waiting to change vector");
  std::this_thread::sleep_for(std::chrono::seconds(26));
  ROS_INFO("3");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  ROS_INFO("2");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  ROS_INFO("1");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  testArray.data = stopVector;
  ROS_INFO("Changed vector");
  std::this_thread::sleep_for(std::chrono::seconds(10));
  testArray.data = goVector;
  ROS_INFO("Changed vector");
  }
}

int main(int argc, char** argv)
{
  testArray.data = goVector;
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  
  test = nh.advertise<std_msgs::Float32MultiArray>("object_detection", 1);
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ControllerConverter Controller(nh, rate);
  DroneControl Drone(nh, rate);

  Drone.ArmDrone = false;
  //Dtest.TakeoffAltitude = 2.5;
  Drone.InitiateTakeoff = false;

  changeVectorThread = std::thread(&WaitThenChangeVector); //Can be used for testing avoidance system
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

        try{
          test.publish(testArray); //Can be used for testing avoidance system
        } catch(ros::Exception &e){
          ROS_INFO_STREAM(e.what());
        }
        Drone.InputTargetPosition = Controller.Target; // Send other input from controller to drone
        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}
