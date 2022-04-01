#include "../include/anti_collision/anti_collision.h"

std::vector<float> analysis::FuzzyGetVelocities(float maxRoll, float maxPitch){
    std::vector<float> velocities = {0,0};
    analysis::engine->process();
    float FuzzyRoll = analysis::Roll->getValue();
    float FuzzyPitch = analysis::Pitch->getValue();

    //Scale Roll
    velocities[1] = (maxRoll/0.5)*FuzzyRoll-maxRoll;
    ROS_INFO("VELOCITITTIES");
    ROS_INFO_STREAM(velocities[1]);
    //Scale Pitch
    velocities[0] = (-maxPitch/0.5)*FuzzyPitch-maxPitch;
    ROS_INFO_STREAM(velocities[0]);

    return velocities;
}

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( cv::contourArea(cv::Mat(contour1)) );
    double j = fabs( cv::contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

void analysis::updateFrame1(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    int dR = img.rows;
    int dC = img.cols;  
    img = img(cv::Range(0,dR),cv::Range(37, dC));
    this->output = img;
}


void analysis::sensorup_update(const std_msgs::String &msg)
{
    sensorUp = std::stof(msg.data);
}

void analysis::sensordown_update(const std_msgs::String &msg)
{
    sensorDown = std::stof(msg.data);
}


void analysis::gridview(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat disparity = cv_bridge::toCvShare(msg, "mono8")->image;
    disparity.convertTo(disparity,CV_8UC1,1.0);
    int N = 60;
    int winCols = disparity.cols;
    int winRows = disparity.rows;
    cv::Mat gridView(winRows,winCols, CV_8UC1);

    for (int r = 0; r < disparity.rows; r += N)
    for (int c = 0; c < disparity.cols; c += N)
    {
        cv::Mat tile = disparity(cv::Range(r, cv::min(r + N, disparity.rows)),
        cv::Range(c, cv::min(c + N, disparity.cols)));//no data copying here

        //tile can be smaller than NxN if image size is not a factor of N
        gridView(cv::Range(r, cv::min(r + N, disparity.rows)),cv::Range(c, cv::min(c + N, disparity.cols))) = cv::mean(tile);
        pubGrid.publish((cv_bridge::CvImage(std_msgs::Header(), "mono8", gridView).toImageMsg()));
    }
}

void analysis::Calibration(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat disparity = cv_bridge::toCvShare(msg, "mono8")->image;
    disparity.convertTo(disparity,CV_8UC1,1.0);
    int r = disparity.rows;
    int c = disparity.cols;
    int N = 20;
    cv::Point TopLeft(((c/2)-N/2), ((r/2)-N/2));
    cv::Point BtmRight(((c/2)+N/2), ((r/2)+N/2));
    cv::Rect rect(((int)((c/2)-N/2)), ((int)((r/2)-N/2))-10, N, N);
    cv::rectangle(disparity, rect, cv::Scalar(0,255,0));
    cv::Mat test = disparity(rect);
    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;

    minMaxLoc(test, &minVal, &maxVal, &minLoc, &maxLoc ); 
    cv::imshow("disparity", disparity);
    cv::imshow("crop", test);
    cv::Scalar meanValue = cv::mean(test);
    double distance = 6646.77f/meanValue[0];
    distance +=  21.669;
    ROS_INFO("-----");
    ROS_INFO("Distance:");
    ROS_INFO_STREAM(distance);
    ROS_INFO("-----");
    ROS_INFO("Max:");
    ROS_INFO_STREAM(maxVal);
    cv::waitKey(1);
}

void analysis::verticalCheck(std::vector<float>& data)
{
    if (sensorUp>minDistRoof){
        data[3] = 0;
    }else{
        data[3] = 1;
    }
    if (sensorDown>minAltitude){
        data[4] = 0;
    }else{
        data[4] = 1;
    }
}

float analysis::CalcVx(double avgDistance){
    Vx = -k1 * std::exp(k3*(k2-(avgDistance*0.01)));
    return Vx;
}


float analysis::CalcVy(cv::Rect b, float Vx)
{
    //  -----Sl----disparity_center----Sr-----  //
    //  -------xl----------------xr-----------  //
    //          ----ll----|--lr---              //
    //return Sl----------------- xr             //
    // Boundaries
    int Sl = disparitry_center - k4;
    int Sr = disparitry_center + k4;
    // position of left and right edge of object
    int xl = b.x; // Left
    int xr = b.x + b.width; // Right
    // edges distance from center
    int ll = disparitry_center - b.x;
    int lr = (b.x+b.width) - disparitry_center;
    ROS_INFO("**CalcVy**");
    ROS_INFO_STREAM(Vx);
    if (xr>Sr && xl<Sl)
    {
        return 0;
    }
    if (ll>lr) // If box is further to the left, ll is longer - best action: Roll right (Positive return value)
    {
        if (xr<Sl) //Object is far enough to the left
        {
            ROS_INFO("CLEAR RIGHT");
            return 0;
        }
        else
        {
            ROS_INFO("ROLLING RIGHT");
            return (xr-Sl)*Vx*k5;
        }
    }
    else if (lr>ll) // If box is further to the right, lr is longer - best action: Roll left (negative return value)
    {
        if (xl>Sr) // Object is far enough to the right
        {
            ROS_INFO("CLEAR LEFT");
            return 0;
        }
        else
        {   
            ROS_INFO("ROLLING LEFT");
            return -(Sr-xl)*Vx*k5;
        }
    }
    else
    {
        return 0;
    }
}

 

void analysis::detectobject(const sensor_msgs::ImageConstPtr& msg)
{
    try
    
        {
            cv::Mat disparity = cv_bridge::toCvShare(msg, "mono8")->image;
            disparity.convertTo(disparity,CV_8UC1,1.0);
            int dR = disparity.rows;
            int dC = disparity.cols;
            disparity = disparity(cv::Range(0,dR),cv::Range(65, dC)); // Remove dead part of frame (37) and slice for equal FOV after
            cv::Mat output_canvas = this->output;
            cv::Mat depth_map = 6646.777f/disparity;
            depth_map +=21.669;
            int sliceIndex = depth_map.cols/3;
            cv::Mat depth_left = depth_map(cv::Range(0,dR),cv::Range(0, sliceIndex));
            cv::Mat depth_mid = depth_map(cv::Range(0,dR),cv::Range(sliceIndex, 2*sliceIndex));
            cv::Mat depth_right = depth_map(cv::Range(0,dR),cv::Range(2*sliceIndex, 3*sliceIndex));
            cv::imshow("Left", depth_left);
            cv::imshow("mid", depth_mid);
            cv::imshow("right", depth_right);
            cv::waitKey(1);
            cv::Mat mask, mean, stddev, mask2, maskL, maskM, maskR;
            // Mask to segment regions with depth less than safe distance
            //cv::inRange(depth_map, 10, depth_thresh, mask);
            cv::inRange(depth_left, 40, depth_thresh, maskL);
            cv::inRange(depth_mid, 40, depth_thresh, maskM);
            cv::inRange(depth_right, 40, depth_thresh, maskR);
            double sL = (cv::sum(maskL)[0])/255.0;
            double sM = (cv::sum(maskM)[0])/255.0;
            double sR = (cv::sum(maskR)[0])/255.0;
            double img_area = double(mask.rows * mask.cols);
            double img_areaL = double(maskL.rows * maskL.cols);
            double img_areaM = double(maskM.rows * maskM.cols);
            double img_areaR = double(maskR.rows * maskR.cols);
            ScreenLS->setValue(sL/img_areaL);
            ScreenMS->setValue(sM/img_areaM);
            ScreenRS->setValue( sR/img_areaR);

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::imshow("maskL", maskL);
            cv::findContours(maskL, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::sort(contours.begin(), contours.end(), compareContourAreas);
            std::vector<cv::Point> cnt = contours[0];

            mask2 = maskL*0;
            cv::drawContours(mask2, contours, 0, (255), -1);
            // Calculating the average depth of the object closer than the safe distance

            cv::meanStdDev(depth_left, mean, stddev, maskL);
            cv::imshow("mask2", mask2);
            ScreenLD->setValue(mean.at<double>(0,0)/60 - 0.66667); // Normalize
            cv::findContours(maskM, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::sort(contours.begin(), contours.end(), compareContourAreas);
            cnt = contours[0];
            cv::drawContours(mask2, contours, 0, (255), -1);
            cv::meanStdDev(depth_mid, mean, stddev, maskM);
            ScreenMD->setValue(mean.at<double>(0,0)/60 - 0.66667);// Normalize
            

            cv::findContours(maskR, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::sort(contours.begin(), contours.end(), compareContourAreas);
            cnt = contours[0];
            
            cv::drawContours(mask2, contours, 0, (255), -1);
            cv::meanStdDev(depth_right, mean, stddev, maskR);
            ScreenRD->setValue(mean.at<double>(0,0)/60 - 0.66667);// Normalize

            std::vector<float> v = analysis::FuzzyGetVelocities(maxRoll, maxPitch);
            data[5] = v[0];
            data[6] = v[1];

            verticalCheck(data);
            detectedObject.data = data;
            pub.publish(detectedObject);


        }
         
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s', to 'bgr8'.", msg->encoding.c_str());
    }
    catch(cv::Exception& e)
    {
        ROS_ERROR("Error: '%s'", msg->encoding.c_str());
    }
    
}


