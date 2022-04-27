#include "../include/anti_collision/anti_collision.h"

void analysis::FuzzyGetVelocities(float maxRoll, float maxPitch){   
    analysis::engine->process();
    float FuzzyRoll = analysis::Roll->getValue();
    float FuzzyPitch = analysis::Pitch->getValue();
    float FuzzyYaw = analysis::Yaw->getValue();
    //ROS_INFO("RAW FUZZY Roll-Pitch-Yaw:");
    //ROS_INFO_STREAM(FuzzyRoll);
    //ROS_INFO_STREAM(FuzzyPitch);
    //ROS_INFO_STREAM(FuzzyYaw);
    //Scale Roll
    data[1] = -1*((maxRoll/0.5)*FuzzyRoll-maxRoll);
    //ROS_INFO("Speed Roll-Pitch-Yaw:");
    //ROS_INFO_STREAM(data[1]);

    //Scale Pitch
    if (FuzzyPitch>0.25f)
    {
        data[0] = -maxPitch*FuzzyPitch;
    }
    else 
    {
        data[0] = 0.0f;
        
    }
    //Scale Yaw
    data[4] = -1*((maxYaw/0.5)*FuzzyYaw-maxYaw);
    //ROS_INFO_STREAM(data[0]);
    //ROS_INFO_STREAM(data[4]);

}

bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( cv::contourArea(cv::Mat(contour1)) );
    double j = fabs( cv::contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

void analysis::updateFrame1(const sensor_msgs::ImageConstPtr& msg)
{
    // NOT IN USE!!
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
    // NOT IN USE!!
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
    // NOT IN USE!!
    cv::Mat disparity = cv_bridge::toCvShare(msg, "mono8")->image;
    disparity.convertTo(disparity,CV_8UC1,1.0);
    int r = disparity.rows;
    int c = disparity.cols;
    int N = 20;
    cv::Point TopLeft(((c/2)-N/2), ((r/2)-N/2));
    cv::Point BtmRight(((c/2)+N/2), ((r/2)+N/2));
    cv::Rect rect(((int)((c/2)-N/2)), ((int)((r/2)-N/2))-10, N, N);
    cv::Rect rect2(((int)((c/2)-N/2)-7), ((int)((r/2)-N/2)-15), N+15, N+15);
    cv::rectangle(disparity, rect2, cv::Scalar(0,255,0));
    cv::Mat test = disparity(rect);
    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;

    minMaxLoc(test, &minVal, &maxVal, &minLoc, &maxLoc ); 
    //cv::imshow("disparity", disparity);
    //cv::imshow("crop", test);
    cv::Scalar meanValue = cv::mean(test);
    ROS_INFO("CALIBRATION (mean, max, min):");

    ROS_INFO_STREAM(meanValue[0]);
    ROS_INFO_STREAM(maxVal);
    ROS_INFO_STREAM(minVal);
    //double distance = 6646.77f/meanValue[0];
    //distance +=  21.669;
    //ROS_INFO("-----");
    //ROS_INFO("Distance:");
    //ROS_INFO_STREAM(distance);
    //ROS_INFO("-----");
    //ROS_INFO("Max:");
    //ROS_INFO_STREAM(maxVal);
    cv::waitKey(1);
}

void analysis::verticalCheck(std::vector<float>& data)
{
    if (sensorUp>minDistRoof){
        data[2] = 0; // Distance form roof is SAFE
    }else{
        data[2] = 1; // Distance from roof is NOT SAFE
    }
    if (sensorDown>minAltitude){
        data[3] = 0; // Distance from floor is SAFE
    }else{
        data[3] = 1; // Distance from floor is too low, recommend increased altitude
    }
}

float analysis::CalcVx(double avgDistance){
    // NOT IN USE!!
    Vx = -k1 * std::exp(k3*(k2-(avgDistance*0.01)));
    return Vx;
}


float analysis::CalcVy(cv::Rect b, float Vx)
{
    // NOT IN USE!!
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


void analysis::displaySystemReaction(cv::Mat dsp, std::vector<float> data)
{
    int cols = dsp.cols;
    int rows = dsp.rows;
    auto center = cv::Point(cols/2, rows/2);
    cv::Point Proll;
    cv::Point Ppitch;
    cv::Point Pyaw;

    int length = 30;
    
    Proll = cv::Point(-data[1]*length, 0);
    Ppitch = cv::Point(0,-data[0]*length);
    Pyaw = cv::Point(-data[4]*length,0);
    
    int lineType = 8;
    int thickness = 3;
    double tipLength = 0.2;
    cv::arrowedLine(dsp, center, center+Proll, CV_RGB(255,0,0),thickness, lineType, 0, tipLength);
    cv::arrowedLine(dsp, center, center+Ppitch, CV_RGB(255,3.1415/2,0),thickness, lineType, 0, tipLength);
    cv::arrowedLine(dsp, center+cv::Point(0,-20), center+Pyaw+cv::Point(0,-20), CV_RGB(255,3.1415/2,0),thickness, lineType, 0, tipLength);
    cv::imshow("ARROW", dsp);
    cv::waitKey(1);
}


void analysis::detectobject(const sensor_msgs::ImageConstPtr& msg)
{
    try
    
        {
            // Acquire and trim disparity map
            cv::Mat disparity = cv_bridge::toCvShare(msg, "mono8")->image;
            disparity.convertTo(disparity,CV_8UC1,1.0);
            int dR = disparity.rows;
            int dC = disparity.cols;
            disparity = disparity(cv::Range(0,dR),cv::Range(65, dC)); // Remove dead part of frame (37) and slice for equal FOV after
            cv::Mat depth_map = 13195.853f/disparity;
            depth_map +=-31.805994;

            // Split into three equal separate depth maps Left(L), Mid(M), Right(R)
            int sliceIndex = depth_map.cols/3;

            int midOffset = 10;
            cv::Mat depth_left = depth_map(cv::Range(50,dR-150),cv::Range(0, sliceIndex));
            cv::Mat depth_mid = depth_map(cv::Range(50,dR-150),cv::Range(sliceIndex-midOffset, 2*sliceIndex+midOffset));
            cv::Mat depth_right = depth_map(cv::Range(50,dR-150),cv::Range(2*sliceIndex, 3*sliceIndex));
            cv::Mat mean, stddev, mask, maskL, maskM, maskR;

            // Mask to segment regions with depth less than safe distance
            cv::inRange(depth_left, depth_min, depth_thresh, maskL);
            cv::inRange(depth_mid, depth_min, depth_thresh, maskM);
            cv::inRange(depth_right, depth_min, depth_thresh, maskR);
            double sL = (cv::sum(maskL)[0])/255.0;
            double sM = (cv::sum(maskM)[0])/255.0;
            double sR = (cv::sum(maskR)[0])/255.0;
            double img_areaL = double(maskL.rows * maskL.cols);
            double img_areaM = double(maskM.rows * maskM.cols);
            double img_areaR = double(maskR.rows * maskR.cols);
            double sizeL = sL/img_areaL;
            double sizeM = sM/img_areaM;
            double sizeR = sR/img_areaR;
            ScreenLS->setValue(sizeL);
            ScreenMS->setValue(sizeM);
            ScreenRS->setValue(sizeR);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            double min, max;
            cv::Point loc1, loc2;
            
            
            //cv::minMaxLoc(depth_left, &min, &max, &loc1, &loc2, maskL);
            // Set parameter on fuzzy regulator
            if (sizeL>0.1)
            {
                cv::meanStdDev(depth_left, mean, stddev, maskL);
                ScreenLD->setValue(mean.at<double>(0,0)*normA + normB); // Normalize
            }
            else
            {
                ScreenLD->setValue(160.0);
            }
            
            
            //ROS_INFO_STREAM("Unnormalized and normalized distances, L, M, R: ");
            //ROS_INFO_STREAM(min);
            //ROS_INFO_STREAM(ScreenLD->getValue());
            // Repeat above step for M and R

            // M
            
            //cv::minMaxLoc(depth_mid, &min, &max, &loc1, &loc2, maskM);
        
            if (sizeM>0.1)
            {
                cv::meanStdDev(depth_mid, mean, stddev, maskM);
                ScreenMD->setValue(mean.at<double>(0,0)*normA + normB);// Normalize [0-1]
            }
            else
            {
                ScreenMD->setValue(160.0);
            }
            
            //ROS_INFO_STREAM(min);
            //ROS_INFO_STREAM(ScreenMD->getValue());

            // L
           
            //cv::minMaxLoc(depth_right, &min, &max, &loc1, &loc2, maskR);
            if (sizeR>0.1)
            {
                cv::meanStdDev(depth_right, mean, stddev, maskR);
                ScreenRD->setValue(mean.at<double>(0,0)*normA + normB);// Normalize [0-1]
            }
            else
            {
                ScreenRD->setValue(160.0);
            }
            
            //ROS_INFO_STREAM(min);
            //ROS_INFO_STREAM(ScreenRD->getValue());
            // Compute
            analysis::FuzzyGetVelocities(maxRoll, maxPitch);
            verticalCheck(data);
            
            // Update message and publish
            detectedObject.data = data;
            pub.publish(detectedObject);
            //cv::imshow("Left", depth_left);
            //cv::imshow("Mid", depth_mid);
            //cv::imshow("Right", depth_right);
            //analysis::displaySystemReaction(disparity, data);
            
            //cv::imshow("L", depth_left);
            //cv::imshow("M", depth_mid);
            //cv::imshow("R", depth_right);
            //cv::imshow("LC", maskL);
            //cv::imshow("MC", maskM);
            //cv::imshow("RC", maskR);
            
            //cv::waitKey(1);


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


