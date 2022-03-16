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

std::vector<std::vector<int>> Waypoints = {
   {1,0,0}, {2,0,0}
};

bool firstLaunch = true;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

nav_msgs::Odometry current_position;
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{ 
   double quatx= msg->pose.pose.orientation.x;
   double quaty= msg->pose.pose.orientation.y;
   double quatz= msg->pose.pose.orientation.z;
   double quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //ROS_INFO("Roll: [%f],Pitch: [%f],Yaw: [%f]",roll,pitch,yaw);
    
  current_position = *msg;
}

bool check_position(geometry_msgs::PoseStamped Setpoint, nav_msgs::Odometry Current, double ErrorTolerance)
{
  double Errorx = Setpoint.pose.position.x - Current.pose.pose.position.x;
  double Errory = Setpoint.pose.position.y - Current.pose.pose.position.y;
  double Errorz = Setpoint.pose.position.z - Current.pose.pose.position.z;

  double magnitude = std::sqrt((Errorx * Errorx) + (Errory * Errory) + (Errorz * Errorz));

  if (magnitude < ErrorTolerance)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool reached_waypoint = false;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ControllerConverter test(nh, rate);
  DroneControl Dtest(nh, rate);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, pose_cb);
  ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
   ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");


  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(test.Target);
    ros::spinOnce();
    rate.sleep();
  }

 mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0; 

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd; 
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (firstLaunch){
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
          firstLaunch = false;
        }
        last_request = ros::Time::now();
      }
    }
  }

    if (test.Land){
      //Initiate landing
      while (!(land_client.call(land_cmd) &&
            land_cmd.response.success) ){
              ROS_INFO("trying to land");
      ros::spinOnce();
      rate.sleep();
    }
      } else {
      local_pos_pub.publish(test.Target);
    }

    //local_pos_pub.publish(test.Target);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
