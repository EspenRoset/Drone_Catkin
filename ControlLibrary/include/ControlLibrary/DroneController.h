#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H
#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include <mavros_msgs/State.h>
#include "geometry_msgs/Twist.h"
#include <mavros_msgs/SetMode.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <vector>
#include <string>
#include <thread>
#include <algorithm>
#include <mutex>
#include <chrono>
#include <condition_variable>

class DroneControl{
    private:
        int state;
        const int Startup = 0;
        const int Takeoff = 1;
        const int Flying = 2;
        const int Landing = 3;
        const int Landed = 4;
        const int AvoidObstacle = 5;
        
        std::chrono::time_point<std::chrono::steady_clock> EndTimeReverse; 
        std::chrono::time_point<std::chrono::steady_clock> EndTimeReset; 
        bool Obstacle_detected = false;
        bool Roof_limit = false;
        bool Floor_limit = false;
        std::vector<float> Obstacle_position;


        ros::Subscriber state_sub;
        ros::Subscriber pos_sub;
        ros::Publisher pos_pub;
        ros::Subscriber collision_sub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient landing_client;
        ros::Rate * rate_;
        ros::Time last_request;

        mavros_msgs::State current_state;
        mavros_msgs::PositionTarget TargetPosition;
        
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        mavros_msgs::CommandTOL land_cmd;

        nav_msgs::Odometry current_position;

        std::thread RunThread;
        std::mutex ArmMutex;
        

        void WaitForFCUConnection();
        void SendNWaipoints(int n);
        void SetPX4Mode(std::string Mode);
        void PX4Arm();
        void DroneTakeoff(float altitude);
        void DroneLand();

    public:
        float TakeoffAltitude = 1.0; // meter
        bool InitiateTakeoff = false;
        bool ArmDrone = false;
        bool InitiateLanding = false;

        std::condition_variable cv;
        mavros_msgs::PositionTarget InputTargetPosition;

    DroneControl(ros::NodeHandle& n, ros::Rate& rate){
        this->rate_ = &rate;
        state = Startup;

        //offb_set_mode.request.custom_mode = "OFFBOARD";
        arm_cmd.request.value = true;

        land_cmd.request.yaw = 0;
        land_cmd.request.latitude = 0;
        land_cmd.request.longitude = 0;
        land_cmd.request.altitude = 0; 


        state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, &DroneControl::state_cb, this);
        pos_sub = n.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, &DroneControl::pose_cb, this);
        pos_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        collision_sub = n.subscribe<std_msgs::Float32MultiArray>("object_detection2", 10, &DroneControl::collision_cb, this);
        arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        landing_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

        TargetPosition.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // Body frame
        TargetPosition.type_mask = 1024; //Ignore Yaw angle, use Yaw_rate

        RunThread = std::thread(&DroneControl::RunDrone, this);
        ROS_INFO("Drone initialized");
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void collision_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void ReverseDrone(int reverseMilliSec);
    void check_height();
    void RunDrone();
};

#endif