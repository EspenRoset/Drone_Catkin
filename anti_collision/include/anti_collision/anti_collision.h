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
    std::vector<float> data{0,0,0,0}; // {vx,vy,fu,fd} vx = pitch, vy = roll, fu = flag up (too close roof), fd = flag down (too close ground)
    float sensorUp;             
    float sensorDown;

    fl::Engine* engine = fl::FllImporter().fromFile("/home/ubuntu/catkin_ws_github/src/anti_collision/src/ObstacleAvoidance.fll"); // Fuzzy engine

    fl::InputVariable* ScreenLS = engine->getInputVariable("ScreenLS"); // Fuzzy inputs
    fl::InputVariable* ScreenLD = engine->getInputVariable("ScreenLD");
    fl::InputVariable* ScreenMS = engine->getInputVariable("ScreenMS");
    fl::InputVariable* ScreenMD = engine->getInputVariable("ScreenMD");
    fl::InputVariable* ScreenRS = engine->getInputVariable("ScreenRS");
    fl::InputVariable* ScreenRD = engine->getInputVariable("ScreenRD");

    fl::OutputVariable* Roll = engine->getOutputVariable("Roll"); // Fuzzy outputs 
    fl::OutputVariable* Pitch = engine->getOutputVariable("Pitch");

    

    public:

    cv::Mat output;

    float minDistRoof = 1500; // Minimum distance above drone  (mm)
    float minAltitude = 500;  // Minimum distance bellow drone (mm)
    float depth_thresh = 105;  // Threshold for SAFE distance (cm)

    // Param Fuzzy
    float maxRoll = 1;
    float maxPitch = 1;


    // Param REVERSE
    float k1 = 1.3; // Max reverse speed
    float k2  = 0.5; // Distance when max reverse is applied
    float k3 = 2.5; // Adjust curve shape -> Higher value = Steeper curve 
    float Vx = 0; //Speed in x direction, to slow down drone
    
    // Param ROLL
    int disparitry_center = 200; // Relative origo disparity map
    float k4 = 50; // Distance from origo considered safe
    float k5 = 0.005; // Gain roll - CAREFUL WITH THIS

    analysis(ros::NodeHandle& nh){
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("/disparity",1 , &analysis::detectobject, this);
        //subClean = it.subscribe("/camera/fisheye1/image_raw/rectified",1 , &analysis::updateFrame1, this);
        subSensorDown = nh.subscribe("/distance_down", 1, &analysis::sensordown_update, this);
        subSensorUp = nh.subscribe("/distance_up", 1, &analysis::sensorup_update, this);
        pub = nh.advertise<std_msgs::Float32MultiArray>("object_detection", 1); 
        //pubGrid = it.advertise("/disparity/gridview",1, &analysis::gridview, this); 

    }

    void sensorup_update(const std_msgs::String &msg);
    void sensordown_update(const std_msgs::String &msg);
    void verticalCheck(std::vector<float>& data);
    void detectobject(const sensor_msgs::ImageConstPtr& msg);
    void gridview(const sensor_msgs::ImageConstPtr& msg);
    void updateFrame1(const sensor_msgs::ImageConstPtr& msg);
    void Calibration(const sensor_msgs::ImageConstPtr& msg);
    float CalcVx(double avgDistance);
    float CalcVy(cv::Rect b, float Vx);
    void FuzzyGetVelocities(float maxRoll, float maxPitch);
    //bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
    


};


#endif
