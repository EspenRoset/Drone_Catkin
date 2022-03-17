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
#include <vector>


void sayHello();

class analysis
{
    cv::Mat rect1;
    cv::Mat rect2;
    cv::Mat output;
    cv::Point minLoc;
    cv::Point maxLoc;
    image_transport::Subscriber sub;
    image_transport::Subscriber subClean;
    image_transport::Subscriber subClean2;
    image_transport::Subscriber subGrid;
    ros::Publisher pub;
    std_msgs::Float32MultiArray detectedObject;
    std::vector<float> data;


    public:
    analysis(ros::NodeHandle& nh){
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("/disparity",1 , &analysis::gridview, this);
        subClean = it.subscribe("/camera/fisheye1/image_raw/rectified",1 , &analysis::updateFrame1, this);
        pub = nh.advertise<std_msgs::Float32MultiArray>("object_detection", 1);  
    }

    void detectobject(const sensor_msgs::ImageConstPtr& msg);
    void gridview(const sensor_msgs::ImageConstPtr& msg);
    void updateFrame1(const sensor_msgs::ImageConstPtr& msg);
    void Calibration(const sensor_msgs::ImageConstPtr& msg);
    //bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
    


};


#endif