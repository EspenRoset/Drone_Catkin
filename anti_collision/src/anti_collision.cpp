#include "../include/anti_collision/anti_collision.h"


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

void analysis::detectobject(const sensor_msgs::ImageConstPtr& msg)
{
    try
    
        {
            cv::Mat disparity = cv_bridge::toCvShare(msg, "mono8")->image;
            disparity.convertTo(disparity,CV_8UC1,1.0);
            int dR = disparity.rows;
            int dC = disparity.cols;
            disparity = disparity(cv::Range(0,dR),cv::Range(37, dC));
            cv::Mat output_canvas = this->output;
            cv::Mat depth_map = 6646.777f/disparity;
            depth_map +=21.669;

            cv::Mat mask, mean, stddev, mask2;
            // Mask to segment regions with depth less than safe distance
            cv::inRange(depth_map, 10, depth_thresh, mask);
            double s = (cv::sum(mask)[0])/255.0;
            double img_area = double(mask.rows * mask.cols);

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
            // Check if a significantly large obstacle is present and filter out smaller noisy regions
            if (s > 0.10*img_area)
            {
                // finding conoturs in the generated mask
                cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                
                // sorting contours from largest to smallest
                std::sort(contours.begin(), contours.end(), compareContourAreas);

                // extracting the largest contour
                std::vector<cv::Point> cnt = contours[0];

                // Check if detected contour is significantly large (to avoid multiple tiny regions)
                double cnt_area = fabs( cv::contourArea(cv::Mat(cnt)));
                if (cnt_area > 0.01*img_area)
                {
                    cv::Rect box;

                    // Finding the bounding rectangle for the largest contour
                    box = cv::boundingRect(cnt);
                    
                    // Update position and flag for detected object
                    float boxX = box.x + box.width/2;
                    float boxY = box.y + box.height/2;
                    
                    data[0] = boxX;
                    
                    data[2] = 1.0f;
                    
                    

                    //finding average depth of region represented by the largest contour
                    mask2 = mask*0;
                    cv::drawContours(mask2, contours, 0, (255), -1);

                    // Calculating the average depth of the object closer than the safe distance
                    //ROS_INFO("PreDev");
                    //replaceInf(depth_map);
                    cv::meanStdDev(depth_map, mean, stddev, mask2);
                    data[1] = mean.at<float>(0,0);
                    //double minVal; 
                    //double maxVal;

                    // Printing the warning text with object distance
                    //char text[10];
                    //ROS_INFO_STREAM(mean.at<double>(0,0));
                    //ROS_INFO_STREAM(mean2);
                    //std::sprintf(text, "%.2f cm",mean.at<double>(0,0));
                    //ROS_INFO_STREAM(depth_map);
                    //cv::putText(output_canvas, "WARNING!", cv::Point2f(box.x + 5, box.y-40), 1, 2, cv::Scalar(0,0,255), 2, 2);
                    //cv::putText(output_canvas, "Object at", cv::Point2f(box.x + 5, box.y), 1, 2, cv::Scalar(0,0,255), 2, 2);
                    //cv::putText(output_canvas, text, cv::Point2f(box.x + 5, box.y+40), 1, 2, cv::Scalar(0,0,255), 2, 2);
                    //ROS_INFO("STRANGER DANGER");
                }
            }
            //else
            //{
                // Printing SAFE if no obstacle is closer than the safe distance
                //cv::putText(output_canvas, "SAFE!", cv::Point2f(200,200),1,2,cv::Scalar(0,255,0),2,2);
                
            //}
            
            verticalCheck(data);
            detectedObject.data = data;
            // Displaying the output of the obstacle avoidance system
            
            //cv::imshow("mask", disparity);
            //cv::imshow("mask2", mask2);
            //cv::imshow("output_canvas",output_canvas);
            pub.publish(detectedObject);
            //cv::waitKey(1);
            ROS_INFO("Vector:");
            ROS_INFO_STREAM(data[0]);
            ROS_INFO_STREAM(data[1]);
            ROS_INFO_STREAM(data[2]);
            ROS_INFO_STREAM(data[3]);
            ROS_INFO_STREAM(data[4]);
        


            
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



