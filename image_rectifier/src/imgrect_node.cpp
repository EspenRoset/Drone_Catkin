#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <sensor_msgs/CameraInfo.h>

class ImageRectifier{
    
    //ros::NodeHandle n;
    image_transport::ImageTransport it_;
    //image_transport::Subscriber image1_sub_;
    //image_transport::Publisher image1_pub_;
    //image_transport::Subscriber image2_sub_;
    //image_transport::Publisher image2_pub_;
    cv::Mat Input;
    cv::Mat Output;
    cv::Mat map1;
    cv::Mat map2;
    cv::Mat undistort;
    
    std::string Substring;
    std::string Pubstring;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    cv::Mat cameraMatrix = cv::Mat(3,3, CV_64F, double(0));
    cv::Mat distortionCoeffs = cv::Mat(1,4, CV_64F, double(0));
    cv::Mat Test = (cv::Mat_<int>(1,2) << 600, 600);
    
    //cv::Mat K1 = (cv::Mat_<double>(3,3) << 285.4117126464844, 0.0, 420.3713073730469, 0.0, 285.50360107421875, 401.1481018066406, 0.0, 0.0, 1.0);
    //cv::Mat D1 = (cv::Mat_<double>(1,4) << -0.0028216259088367224, 0.033221110701560974, -0.030459459871053696, 0.0038244719617068768);
    

public:
    sensor_msgs::Image img_msg; // >> message to be sent
    ImageRectifier(std::string CameraSelect, ros::NodeHandle n)
     : it_(n){
        Substring = "/camera/fisheye"+CameraSelect+"/image_raw";
        Pubstring = "/image_rectified/image"+CameraSelect;
        image_sub_ = it_.subscribe(Substring, 1, &ImageRectifier::imageCb, this);
        image_pub_ = it_.advertise(Pubstring, 1);

    /*
        image1_sub_ = it_.subscribe("/camera/fisheye1/image_raw", 1, &ImageRectifier::imageCb, this);
        image2_sub_ = it_.subscribe("/camera/fisheye2/image_raw", 1, &ImageRectifier::imageCb, this);
        image1_pub_ = it_.advertise("/image_rectified/left", 1);
        image2_pub_ = it_.advertise("/image_rectified/right", 1);
*/
    //std::vector<image_transport::Publisher> PubVector = {image1_pub_, image2_pub_};

    cameraMatrix.at<double>(0, 0) = 285.4117126464844;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 420.3713073730469;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 285.50360107421875;
    cameraMatrix.at<double>(1, 2) = 401.1481018066406;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;

    distortionCoeffs.at<double>(0,0) = -0.0028216259088367224;
    distortionCoeffs.at<double>(0,1) = 0.033221110701560974;
    distortionCoeffs.at<double>(0,2) = -0.030459459871053696;
    distortionCoeffs.at<double>(0,3) = 0.0038244719617068768;

   //cv::Size size = {Input.cols, Input.rows};
    cv::Size size = {848, 800};
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distortionCoeffs, cv::Mat::eye(3,3, cv::DataType<double>::type), cameraMatrix, size, CV_16SC2, map1, map2);

    }

   



    cv_bridge::CvImagePtr cv_ptr;

~ImageRectifier(){
}

void imageCb(const sensor_msgs::ImageConstPtr& msg){

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Input = cv_ptr->image;
    cv::Size size = {Input.cols, Input.rows};
    ROS_INFO_STREAM(size);
    //cv::fisheye::initUndistortRectifyMap(cameraMatrix, distortionCoeffs, cv::Mat::eye(3,3, cv::DataType<double>::type), cameraMatrix, size, CV_16SC2, map1, map2);
    cv::remap(Input, Output, map1, map2, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);

    //cv::fisheye::undistortImage( Input, Output, cameraMatrix, distortionCoeffs, cv::Mat::eye(3,3,CV_64F), size);
    
    cv_ptr->image = Output;
    
    //cv_ptr = cv_brigde::toImageMsg(Output, sensor_msgs::image_encodings::BGR8)//image_pub_.publish(cv_ptr->toImageMsg());
    image_pub_.publish(cv_ptr->toImageMsg());
    }
};



int main(int argc, char **argv){
    ros::init(argc, argv, "image_rectifier");  
    ros::NodeHandle n;
    ros::Rate rate(30);
    ImageRectifier ic1("1", n);
    ImageRectifier ic2("2", n);
    ros::spin();
    return 0;
}