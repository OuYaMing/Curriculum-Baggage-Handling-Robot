/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "Viewer.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <thread>
#include <future>
#include "kcf_follower/flag.h"
#include "kcf_follower/voice.h"
using namespace std;




//void th1()
//{
//
//}
//void th2()
//{
//	while (1)
//	{
//		cout<<"2222"<<'\n';
//		ros::Duration(1).sleep();
//	}
//
//
//}

int main(int argc, char** argv)
{
    openni::Status rc = openni::STATUS_OK;
    SampleViewer sampleViewer("User Viewer");
	rc = sampleViewer.Init(argc, argv);
	ros::init(argc, argv, "main");
	ros::NodeHandle service_handle;
	ros::ServiceServer service = service_handle.advertiseService("get_depth", &SampleViewer::get_depth_handle_function, &sampleViewer);



	// ros::Publisher statePub = nh.advertise<kcf_follower::voice>("state_flag",1);
	// kcf_follower::flag state;
	// state.init_tracking = false;
	// state.start_follow = false;
	// statePub.publish(state);


	if (rc != openni::STATUS_OK)
	{
		return 1;
	}


//	std::thread t1(th1);
//	std::thread t2(th2);
//	t1.join();
//	t2.join();
	//sampleViewer.Run();
    while(true){
        sampleViewer.Display();

        ros::spinOnce();
    }

}
