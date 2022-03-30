#ifndef ANTI_COLLISION_H
#define ANTI_COLLISION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <stdio.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <vector>


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
    std::vector<float> data{0,0,0,0,0}; // {x,y,f,fu,fd} x,y = location of object | f = 1 object is too close 
                                        //               fu = 1 to high           | fd = to low
    cv::Mat output;
    float sensorUp;             
    float sensorDown;          
    

    public:

    float minDistRoof = 1500; // Minimum distance above drone  (mm)
    float minAltitude = 500;  // Minimum distance bellow drone (mm)
    float depth_thresh = 105;  // Threshold for SAFE distance (cm)

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
    //bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
    


};


#endif
