#!/usr/bin/env python2
# coding:utf-8

# 上面的第二句指定编码类型为utf-8，是为了使python能够识别中文

# 加载所需模块
import rospy
import cv2
from kcf_follower.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def client_srv():
    rospy.init_node('client')
    while(1):
        rospy.wait_for_service("get_depth")
        try:
            # 定义service客户端，service名称为“greetings”，service类型为Greeting
            greetings_client = rospy.ServiceProxy("get_depth",depth)

            # 向server端发送请求，发送的request内容为name和age,其值分别为"HAN", 20
            # 注意，此处发送的request内容与service文件中定义的request部分的属性是一致的
            #resp = greetings_client("HAN",20)
            resp = greetings_client.call(100,100)

            # 打印处理结果，注意调用response的方法，类似于从resp对象中调取response属性
            rospy.loginfo("Message From server:%s"%resp.feedback)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed: %s"%rospy.ServiceException)


# 如果单独运行此文件，则将上面函数client_srv()作为主函数运行
if __name__=="__main__":
    client_srv()
