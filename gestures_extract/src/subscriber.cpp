
#include <ros/ros.h>

#include<gestures_extract/body.h>
#include "std_msgs/String.h" 

void chatterCallback(const gestures_extract::body::ConstPtr& msg)               //std_msgs::String
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f    %f     %f]",msg->x, msg->y, msg->z); //接收的消息
    //ROS_INFO("I heard");
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"example_subscriber");    //a)
    ros::NodeHandle n;                            //b)
    ros::Subscriber sub = n.subscribe("message",1000,chatterCallback);    //c)
    ros::spin();                                  //回调函数
    return 0;
}
/*
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_subscriber");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/color_image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}*/