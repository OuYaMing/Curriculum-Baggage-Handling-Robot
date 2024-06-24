#include <stdlib.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
//包含自定义msg产生的头文件
#include <voice/voice.h>

using namespace std;

void voiceBack(const voice::voice::ConstPtr &msg)
{
    string playfile = string("play ") + msg->filename;
    ROS_INFO("Now play: %s", playfile.c_str());
    system(playfile.c_str());
    if(msg->filename == "/home/xjp/turtlebot_ws/src/kcf_follower/voice/bye.mp3")
    {
        ros::shutdown();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "speak");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("voice_speak",1,voiceBack);

    ros::spin();
    return 0;
}