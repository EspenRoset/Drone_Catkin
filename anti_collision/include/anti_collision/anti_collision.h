#ifndef ANTI_COLLISION_H
#define ANTI_COLLISION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <fl/Headers.h>
#include <stdio.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <vector>
#include <math.h>

void sayHello();

class analysis
{
    image_transport::Subscriber sub;
    image_transport::Subscriber subClean;
    image_transport::Subscriber subClean2;
    image_transport::Subscriber subGrid;
    ros::Subscriber subSensorUp;
    ros::Subscriber subSensorDown;
    ros::Publisher pub;
    image_transport::Publisher pubGrid;

    std_msgs::Float32MultiArray detectedObject;
    std::vector<float> data{0,0,0,0,0}; // {vx,vy,fu,fd, yaws} vx = pitch, vy = roll, fu = flag up (too close roof), fd = flag down (too close ground)
    float sensorUp;             
    float sensorDown;

    fl::Engine* engine = fl::FllImporter().fromFile("/home/ubuntu/catkin_ws_github/src/anti_collision/src/ObstacleAvoidance5.fll"); // Fuzzy engine

    fl::InputVariable* ScreenLS = engine->getInputVariable("ScreenLS"); // Fuzzy inputs
    fl::InputVariable* ScreenLD = engine->getInputVariable("ScreenLD");
    fl::InputVariable* ScreenMS = engine->getInputVariable("ScreenMS");
    fl::InputVariable* ScreenMD = engine->getInputVariable("ScreenMD");
    fl::InputVariable* ScreenRS = engine->getInputVariable("ScreenRS");
    fl::InputVariable* ScreenRD = engine->getInputVariable("ScreenRD");

    fl::OutputVariable* Roll = engine->getOutputVariable("Roll"); // Fuzzy outputs 
    fl::OutputVariable* Pitch = engine->getOutputVariable("Pitch");
    fl::OutputVariable* Yaw = engine->getOutputVariable("Yaw");

    

    public:

    cv::Mat output;

    // Param Safety
    float minDistRoof = 1500; // Minimum distance above drone  (mm)
    int roofCount = 0; // For filtering bad data from sensor
    float minAltitude = 500;  // Minimum distance bellow drone (mm)
    float depth_thresh = 160;  // Threshold for SAFE distance (cm)
    float depth_min = 30; // Threshold for closest distance estimated (cm)
    float normA = 1/(depth_thresh-depth_min);
    float normB = -depth_min/(depth_thresh-depth_min);
    int hCut = 50; // Amount of rows to remove from bottom of depth map

    // Param Fuzzy
    float maxRoll = 1;
    float maxPitch = 1;
    float maxYaw = 1;


    // Param REVERSE // NOT IN USE!!
    float k1 = 1.3; // Max reverse speed
    float k2  = 0.5; // Distance when max reverse is applied
    float k3 = 2.5; // Adjust curve shape -> Higher value = Steeper curve 
    float Vx = 0; //Speed in x direction, to slow down drone
    
    // Param ROLL // NOT IN USE!!
    int disparitry_center = 200; // Relative origo disparity mapz|
    float k4 = 50; // Distance from origo considered safe
    float k5 = 0.005; // Gain roll - CAREFUL WITH THIS

    analysis(ros::NodeHandle& nh){
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("/disparity",1 , &analysis::detectobject, this);
        //sub = it.subscribe("/disparity",1 , &analysis::Calibration, this); // FOR CALIBARTION, should be commented out unless calibrating
        //subClean = it.subscribe("/camera/fisheye1/image_raw/rectified",1 , &analysis::updateFrame1, this);
        subSensorDown = nh.subscribe("/distance_down", 1, &analysis::sensordown_update, this);
        subSensorUp = nh.subscribe("/distance_up", 1, &analysis::sensorup_update, this);
        pub = nh.advertise<std_msgs::Float32MultiArray>("course_correction", 1); 
        //pubGrid = it.advertise("/disparity/gridview",1, &analysis::gridview, this); 

    }

    void sensorup_update(const std_msgs::String &msg); // IN USE
    void sensordown_update(const std_msgs::String &msg); // IN USE
    void verticalCheck(std::vector<float>& data); // IN USE
    void detectobject(const sensor_msgs::ImageConstPtr& msg); // IN USE
    void gridview(const sensor_msgs::ImageConstPtr& msg); // NOT
    void updateFrame1(const sensor_msgs::ImageConstPtr& msg); // NOT
    void Calibration(const sensor_msgs::ImageConstPtr& msg); // NOT
    float CalcVx(double avgDistance); // NOT
    float CalcVy(cv::Rect b, float Vx); //NOT
    void FuzzyGetVelocities(float maxRoll, float maxPitch); // IN USE
    void displaySystemReaction(cv::Mat dsp, std::vector<float> data);
};


#endif
