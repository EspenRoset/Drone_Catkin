#ifndef CONTROLLERCONVERTER_H
#define CONTROLLERCONVERTER_H
#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include <vector>
#include <thread>
#include <algorithm>

void sayHello();

class ControllerConverter{
    private:
    ros::Subscriber ControlSub;
    ros::Subscriber PosSub;
    ros::Rate * rate_;
    void CalcYaw(float yaw);
    float Z = 0;
    bool YawSet = false;
    float Yaw_Temp_ = 0;
    float Yaw_ = 0;
    float Vx_ = 0;
    float Vz_ = 0;
    float Vmultiplier = 0;
    int landBtn = 0;
    int armingBtn = 0;
    int takeoffBtn = 0;
    int returnBtn = 0;
    std::thread UpdateThread;


    public:
    bool Land = false;
    bool Takeoff = false;
    bool Arm = false;
    bool Return = false;
    mavros_msgs::PositionTarget Target; 

    ControllerConverter(ros::NodeHandle& n, ros::Rate& rate){
        this->rate_ = &rate;
        ControlSub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &ControllerConverter::ControlUpdate, this);
        PosSub = n.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, &ControllerConverter::pose_cb, this);
        Target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        UpdateThread = std::thread(&ControllerConverter::UpdateVelocities, this);
        Target.type_mask = 1024;
    }
    std::vector<float> Scaling = {0.5, 1, 0.5, 2}; 
    // Pitch, Yaw, Throttle

    void ControlUpdate(const sensor_msgs::JoyConstPtr& msg);
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void UpdateVelocities();

};

#endif