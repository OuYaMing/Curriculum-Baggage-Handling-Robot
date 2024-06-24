/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#include "NiTE.h"
#include <OpenNI.h>


#include <pcl/common/common_headers.h> // for pcl::PointCloud
#include <pcl/visualization/pcl_visualizer.h>
#include<kcf_follower/person.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include "kcf_follower/depth.h"

#define MAX_DEPTH 10000




class SampleViewer
{
public:
	SampleViewer(const char* strSampleName);
	virtual ~SampleViewer();
    virtual void Display();
	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return
 	openni::VideoStream mColorStream;
    openni::VideoStream mDepthStream;
	bool get_depth_handle_function(kcf_follower::depth::Request &req,kcf_follower::depth::Response &res);
	// ros::NodeHandle *service_handle;
	// ros::ServiceServer *get_depth_service;// = n->advertiseService("get_depth", get_depth_handle_function);
    void SendFlag(bool init_tracking, bool start_follow);

protected:
//	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenGL(int argc, char **argv);
	void InitOpenGLHooks();

    bool getCloudXYZCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB, openni::VideoFrameRef& depthFrame);
    bool SendMsg(const nite::UserData &user, sensor_msgs::ImagePtr color_msg, int flag, openni::VideoFrameRef depthFrame);
    bool localization_bag(nite::UserTracker *pUserTracker, const nite::UserData &user,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB, pcl::PointXYZ& hp, pcl::PointXYZ& hv);
	
	void Finalize();

private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	openni::Device		m_device;
	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;
	nite::HandTracker mHandTracker;



};


#endif // _NITE_USER_VIEWER_H_
