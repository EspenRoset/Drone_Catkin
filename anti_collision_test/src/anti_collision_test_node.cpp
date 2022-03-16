#include <ros/ros.h>
#include "anti_collision/anti_collision.h"
// Githubtest
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::Rate rate(20.0);
  analysis test(nh);

  ros::spin();
  rate.sleep();
}