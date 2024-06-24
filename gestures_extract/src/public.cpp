
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
int main(int argc,char **argv)
{
    ros::init(argc,argv,"example_public");       //a)
    ros::NodeHandle n;                           //b)
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message",1000);    //c)
    ros::Rate loop_rate(10);                     //d)
    while(ros::ok())                     //中断，遇到ctrl+c时候中断
    {
        std_msgs::String msg;            //申请内存
        std::stringstream ss;                  //信息内容变量
        ss << "I am publisher ";         //写入变量
        msg.data = ss.str();             //写入内存
        chatter_pub.publish(msg);        //调用发布函数
        loop_rate.sleep( );               //发送频率
    }
    return 0;
}


