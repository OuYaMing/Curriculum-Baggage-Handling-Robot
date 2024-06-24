/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS

#include <inttypes.h>

#endif

#include "Viewer.h"
/*
#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif
*/
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut_std.h>
#include "NiteSampleUtilities.h"
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sstream>

#include<gestures_extract/body.h>
#include<kcf_follower/person.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include "kcf_follower/depth.h"
#include "kcf_follower/flag.h"
#include "kcf_follower/voice.h"
#include "seg_obj.h"

#define GL_WIN_SIZE_X    640//640//
#define GL_WIN_SIZE_Y    480//480//
#define TEXTURE_SIZE    512

#define DEPTH_SIZE_X 640
#define DEPTH_SIZE_Y 480
#define COLOR_SIZE_X 1920
#define COLOR_SIZE_Y 1080


#define DEFAULT_DISPLAY_MODE    DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)    ((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)    (MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))


bool is_out_of_scene =  false;

bool cloud_point = false;
//bool have_cal_bag = false;
bool start_tracking = false;
bool init_tracking = true;

bool now_is_goodbye = false;

using namespace std;
SampleViewer *SampleViewer::ms_self = NULL;


using namespace openni;
using namespace nite;


bool g_drawSkeleton = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel = true;
bool g_drawBoundingBox = true;
bool g_drawBackground = true;
bool g_drawDepth = true;
bool g_drawFrameId = false;

int g_nXRes = 0, g_nYRes = 0;

struct MyVector {
    int x;
    int y;
    int z;
};

ros::NodeHandle *n;                           //b)
ros::Publisher *chatter_pub;

ros::NodeHandle *state_handle;  
ros::Publisher *statePub;


ros::NodeHandle *color_n;                           //b)
image_transport::Publisher color_pub;

ros::NodeHandle *depth_n;                           //b)
image_transport::Publisher depth_pub;

ros::NodeHandle *Voice_n;
ros::Publisher *voicePub;


//ros::NodeHandle *service_handle;
//ros::ServiceServer *service;


// time to hold in pose to exit program. In milliseconds.
const int g_poseTimeoutToExit = 2000;

bool SampleViewer::get_depth_handle_function(kcf_follower::depth::Request &req,
					kcf_follower::depth::Response &res)
{
	ROS_INFO("x:%d y: %d ", req.x, req.y);
    openni::VideoFrameRef depthFrame;
    openni::VideoFrameRef colorFrame;
    SampleViewer::mDepthStream.readFrame(&depthFrame);
    SampleViewer::mColorStream.readFrame(&colorFrame);

    int rgb_x = req.x;
    int rgb_y = req.y;



	res.depth = 2500;
	return true;
}

void SampleViewer::glutIdle() {
    glutPostRedisplay();
}

void SampleViewer::glutDisplay() {
    SampleViewer::ms_self->Display();
}

void SampleViewer::glutKeyboard(unsigned char key, int x, int y) {
    SampleViewer::ms_self->OnKey(key, x, y);
}

SampleViewer::SampleViewer(const char *strSampleName) : m_poseUser(0) {
    ms_self = this;
    strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
    m_pUserTracker = new nite::UserTracker;
}

SampleViewer::~SampleViewer() {
    Finalize();

    delete[] m_pTexMap;

    ms_self = NULL;
}

void SampleViewer::Finalize() {
    mColorStream.destroy();

    mDepthStream.destroy();

        // 关闭HandTracker跟踪
    mHandTracker.destroy();

    delete m_pUserTracker;
    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
}


openni::Status SampleViewer::Init(int argc, char **argv) {


    cout<<" now,we are initing!"<<endl;

    m_pTexMap = NULL;

    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK) {
        printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
        return rc;
    }

    const char *deviceUri = openni::ANY_DEVICE;
    for (int i = 1; i < argc - 1; ++i) {
        if (strcmp(argv[i], "-device") == 0) {
            deviceUri = argv[i + 1];
            break;
        }
    }

    rc = m_device.open(deviceUri);
    if (rc != openni::STATUS_OK) {
        printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
        return rc;
    }

    m_device.setDepthColorSyncEnabled(true);

    mDepthStream.create(m_device, SENSOR_DEPTH);
    VideoMode mDepthMode;
    mDepthMode.setResolution(1280, 960);
    mDepthMode.setFps(30);
    mDepthMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    mDepthStream.setVideoMode(mDepthMode);


    mColorStream.create(m_device, SENSOR_COLOR);
    VideoMode mColorMode;
    mColorMode.setResolution(1280, 960);
    mColorMode.setFps(30);
    mColorMode.setPixelFormat(PIXEL_FORMAT_RGB888);
    mColorStream.setVideoMode(mColorMode);

    // 设置深度图像映射到彩色图像
//    m_device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );




//    m_device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );

    nite::NiTE::initialize();

    if (m_pUserTracker->create(&m_device) != nite::STATUS_OK) {
        return openni::STATUS_ERROR;
    }



        // 创建HandTracker跟踪器
    mHandTracker.create();

    // 设定手势探测（GESTURE_WAVE、GESTURE_CLICK和GESTURE_HAND_RAISE）
   mHandTracker.startGestureDetection( GESTURE_WAVE );
//   mHandTracker.startGestureDetection( GESTURE_CLICK );
//   mHandTracker.startGestureDetection( GESTURE_HAND_RAISE );


    ros::init(argc, argv, "example_public");       //a)

    n = new(ros::NodeHandle);                           //b)
    chatter_pub = new(ros::Publisher);
    *chatter_pub = n->advertise<kcf_follower::person>("message", 1);    //c

    Voice_n = new(ros::NodeHandle);
    voicePub =  new(ros::Publisher);
    *voicePub = Voice_n->advertise<kcf_follower::voice>("voice_speak",1);

//    kcf_follower::voice voiceFile;
//    voiceFile.filename = "/home/xjp/turtlebot_ws/src/kcf_follower/voice/xiaoI.mp3";
//    voicePub->publish(voiceFile);

    //ros::Duration(5).sleep();

//    ros::init(argc, argv, "main");
    //service_handle = new(ros::NodeHandle);
    //*service = service_handle->advertiseService("get_depth", &SampleViewer::get_depth_handle_function, this);

    state_handle = new(ros::NodeHandle);  
    statePub = new(ros::Publisher);
    *statePub = state_handle->advertise<kcf_follower::flag>("state_flag",1);



    color_n = new(ros::NodeHandle);
    image_transport::ImageTransport color_it(*color_n);
    color_pub = color_it.advertise("camera/color_image", 1);


    depth_n = new(ros::NodeHandle);
    image_transport::ImageTransport depth_it(*depth_n);
    depth_pub = depth_it.advertise("camera/depth_image", 1);



    mColorStream.start();
    mDepthStream.start();


    return InitOpenGL(argc, argv);

}

openni::Status SampleViewer::Run()    //Does not return
{

    glutMainLoop();

    return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0},
                     {0, 1, 0},
                     {0, 0, 1},
                     {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
    sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
    printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}

void updateUserState(const nite::UserData &user, uint64_t ts) {
    if (user.isNew()) {
        USER_MESSAGE("New");
    } else if (user.isVisible() && !g_visibleUsers[user.getId()])
        printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
    else if (!user.isVisible() && g_visibleUsers[user.getId()])
        printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
    else if (user.isLost()) {
        USER_MESSAGE("Lost");
    }
    g_visibleUsers[user.getId()] = user.isVisible();


    if (g_skeletonStates[user.getId()] != user.getSkeleton().getState()) {
        switch (g_skeletonStates[user.getId()] = user.getSkeleton().getState()) {
            case nite::SKELETON_NONE: USER_MESSAGE("Stopped tracking.")
                break;
            case nite::SKELETON_CALIBRATING: USER_MESSAGE("Calibrating...")
                break;
            case nite::SKELETON_TRACKED: USER_MESSAGE("Tracking!")
                break;
            case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
            case nite::SKELETON_CALIBRATION_ERROR_HANDS:
            case nite::SKELETON_CALIBRATION_ERROR_LEGS:
            case nite::SKELETON_CALIBRATION_ERROR_HEAD:
            case nite::SKELETON_CALIBRATION_ERROR_TORSO: USER_MESSAGE("Calibration Failed... :-|")
                break;
        }
    }
}

#ifndef USE_GLES

void glPrintString(void *font, const char *str) {
    int i, l = (int) strlen(str);

    for (i = 0; i < l; i++) {
        glutBitmapCharacter(font, *str++);
    }
}

#endif

void DrawStatusLabel(nite::UserTracker *pUserTracker, const nite::UserData &user) {
    int color = user.getId() % colorCount;
    glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

    float x, y;
    pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y,
                                                 user.getCenterOfMass().z, &x, &y);
    x *= GL_WIN_SIZE_X / (float) g_nXRes;
    y *= GL_WIN_SIZE_Y / (float) g_nYRes;
    char *msg = g_userStatusLabels[user.getId()];
    glRasterPos2i(x - ((strlen(msg) / 2) * 8), y);
    glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}

void DrawFrameId(int frameId) {
    char buffer[80] = "";
    sprintf(buffer, "%d", frameId);
    glColor3f(1.0f, 0.0f, 0.0f);
    glRasterPos2i(20, 20);
    glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}

void DrawCenterOfMass(nite::UserTracker *pUserTracker, const nite::UserData &user) {
    glColor3f(1.0f, 1.0f, 1.0f);

    float coordinates[3] = {0};

    pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y,
                                                 user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);

    coordinates[0] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[1] *= GL_WIN_SIZE_Y / (float) g_nYRes;
    glPointSize(8);
    glVertexPointer(3, GL_FLOAT, 0, coordinates);
    glDrawArrays(GL_POINTS, 0, 1);

}

void DrawBoundingBox(const nite::UserData &user) {
    glColor3f(1.0f, 1.0f, 1.0f);

    float coordinates[] =
            {
                    user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
                    user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
                    user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
                    user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
            };

    //cout<<"Box_x                 "<<user.getBoundingBox().max.x<<"     Box_y          "<<user.getBoundingBox().max.y<<endl;


    coordinates[0] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[1] *= GL_WIN_SIZE_Y / (float) g_nYRes;
    coordinates[3] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[4] *= GL_WIN_SIZE_Y / (float) g_nYRes;
    coordinates[6] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[7] *= GL_WIN_SIZE_Y / (float) g_nYRes;
    coordinates[9] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[10] *= GL_WIN_SIZE_Y / (float) g_nYRes;

    glPointSize(2);
    glVertexPointer(3, GL_FLOAT, 0, coordinates);
    glDrawArrays(GL_LINE_LOOP, 0, 4);

}


void DrawLimb(nite::UserTracker *pUserTracker, const nite::SkeletonJoint &joint1, const nite::SkeletonJoint &joint2,
              int color) {
    float coordinates[6] = {0};
    pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z,
                                                 &coordinates[0], &coordinates[1]);

    //相机坐标系坐标
    //cout<<"jointx  "<<joint1.getPosition().x<<"  jointy  "<<joint1.getPosition().y<<"  jointz  "<<joint1.getPosition().z<<endl;

    //图像坐标系：像素点坐标
    //cout<<"Box_x                 "<<coordinates[0]<<"     Box_y          "<<coordinates[1]<<endl;

    pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z,
                                                 &coordinates[3], &coordinates[4]);



    coordinates[0] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[1] *= GL_WIN_SIZE_Y / (float) g_nYRes;
    coordinates[3] *= GL_WIN_SIZE_X / (float) g_nXRes;
    coordinates[4] *= GL_WIN_SIZE_Y / (float) g_nYRes;

    if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1) {
        glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
    } else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f) {
        return;
    } else {
        glColor3f(.5, .5, .5);
    }
    glPointSize(2);
    glVertexPointer(3, GL_FLOAT, 0, coordinates);
    glDrawArrays(GL_LINES, 0, 2);

    glPointSize(10);
    if (joint1.getPositionConfidence() == 1) {
        glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
    } else {
        glColor3f(.5, .5, .5);
    }
    glVertexPointer(3, GL_FLOAT, 0, coordinates);
    glDrawArrays(GL_POINTS, 0, 1);

    if (joint2.getPositionConfidence() == 1) {
        glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
    } else {
        glColor3f(.5, .5, .5);
    }
    glVertexPointer(3, GL_FLOAT, 0, coordinates + 3);
    glDrawArrays(GL_POINTS, 0, 1);
}

void DrawSkeleton(nite::UserTracker *pUserTracker, const nite::UserData &userData) {
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD),
             userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
             userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW),
             userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
             userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
             userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO),
             userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);


    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
             userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE),
             userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE),
             userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}

int orderCnt = 0;
int order = -1;
int userID = -1;
MyVector right_vec1, right_vec2;
MyVector left_vec1, left_vec2;


int argc;
char **argv;



//openni图像流转化成点云
pcl::visualization::CloudViewer cloud_viewer("Cloud Viewer");
pcl::PointXYZ hole_point(0.,0.,0.),hole_vec(0.,0.,0.);
pcl::PointXYZ tmpp1,tmpp2;
pcl::PointXY tmpp3,tmpp4;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 0,0);
    viewer.setCameraPosition(0, 0, 0, 0, 1, -1, 0);
}
void show_hole(pcl::visualization::PCLVisualizer& viewer){
    if(hole_point.z!=0) {
        viewer.removeAllShapes();
        viewer.addArrow(hole_point, hole_vec, 1, 1, 0, "arrow");
        cout << "hole_point:" << hole_point << " , hole_vec:" << hole_vec << endl;
        hole_point = pcl::PointXYZ(0,0,0);
        
    }else{

    }
}
void showVec (pcl::visualization::PCLVisualizer& viewer){
    viewer.removeAllShapes();
    viewer.addArrow(tmpp1,tmpp2,1,1,0);
}

bool SampleViewer::getCloudXYZCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB, openni::VideoFrameRef& depthFrame){
    float fx, fy, fz;
    double fScale = 0.001;
    int dh = depthFrame.getHeight();
    int dw = depthFrame.getWidth();
    cloud_XYZRGB->is_dense = false;
    openni::DepthPixel *pDepthArray = (openni::DepthPixel *) depthFrame.getData();
//        cv::Mat im(480,640,CV_8UC1, cv::Scalar(0));
    const float not_num = std::numeric_limits<float>::quiet_NaN();
    for (int y = 0; y < dh; y++) {
        for (int x = 0; x < dw; x++) {
            int idx = x + y * dw;
            const openni::DepthPixel rDepth = pDepthArray[idx];

            openni::CoordinateConverter::convertDepthToWorld(mDepthStream, x, y, rDepth, &fx, &fy, &fz);
//                fx = -fx;
//                fy = -fy;
            if( fz==0){
                cloud_XYZRGB->points[idx].x = not_num;
                cloud_XYZRGB->points[idx].y = not_num;
                cloud_XYZRGB->points[idx].z = not_num;
                cloud_XYZRGB->points[idx].r=cloud_XYZRGB->points[idx].g=cloud_XYZRGB->points[idx].b = 0;
            }else {
                cloud_XYZRGB->points[idx].x = fx * fScale;
                cloud_XYZRGB->points[idx].y = fy * fScale;
                cloud_XYZRGB->points[idx].z = fz * fScale;
                cloud_XYZRGB->points[idx].r = 255;
                cloud_XYZRGB->points[idx].g = 255;
                cloud_XYZRGB->points[idx].b = 255;
            }
        }
    }
    return true;
}


int limit_x(int x)
{
    x = x<0?0:x;
    x = x>640?640:x;
    return x;
}

int limit_y(int y)
{
    y = y<0?0:y;
    y = y>480?480:y;
    return y;
}

bool SampleViewer::SendMsg(const nite::UserData &user, sensor_msgs::ImagePtr color_msg, int flag, openni::VideoFrameRef depthFrame) {
    kcf_follower::person person_msg;
    person_msg.rgb_image = *color_msg;



    const openni::DepthPixel *pdepth = (DepthPixel*)depthFrame.getData();
    int depth_minx = user.getBoundingBox().min.x;
    int depth_miny = user.getBoundingBox().min.y;
    int depth_maxx = user.getBoundingBox().max.x;
    int depth_maxy = user.getBoundingBox().max.y;

//    int centerbody_depth = user.
    int min_depth = 10000;
    for (int y = 100; y < 240; y++) {
        for (int x = 240; x < 380; x++) {

            float fx, fy, fz;
            int idx = x + y * 640;
            const openni::DepthPixel rDepth = pdepth[idx];
            openni::CoordinateConverter::convertDepthToWorld(mDepthStream, x, y, rDepth, &fx, &fy, &fz);
            if (min_depth > fz && fz>0)
                min_depth = (int) fz;
        }
    }


    depth_minx = limit_x(depth_minx);
    depth_maxx=limit_x(depth_maxx);

    depth_miny = limit_y(depth_miny);
    depth_maxy=limit_y(depth_maxy);

    int color_minx = 0;
    int color_miny = 0;
    int color_maxx = 0;
    int color_maxy = 0;

    DepthPixel depthvmin = pdepth[depth_miny*depthFrame.getWidth() + depth_minx];
    DepthPixel depthvmax = pdepth[depth_maxy*depthFrame.getWidth() + depth_maxx];

    openni::CoordinateConverter::convertDepthToColor(mDepthStream,mColorStream,depth_minx,depth_miny,1500,&color_minx,&color_miny);
    openni::CoordinateConverter::convertDepthToColor(mDepthStream,mColorStream,depth_maxx,depth_maxy,1500,&color_maxx,&color_maxy);




    int color_width = color_maxx - color_minx;
    int color_height = color_maxy - color_miny;




    //获取深度图box中心像素
    int depth_x = (depth_minx + depth_maxx)/2;
    int depth_y = (depth_miny + depth_maxy)/2;

  //  cout<<"  depth_minx:"<<depth_minx<<"  depth_miny:"<<depth_miny<<"  depth_maxx:"<<depth_maxx<<"  depth_maxy:"<<depth_maxy<<endl;


 //   cout<<"  color_minx"<<color_minx<<" color_miny"<<color_miny<<"  color_width"<<color_width<<"  color_height"<<color_height<<endl;

    person_msg.xMin = color_minx;
    person_msg.yMin = color_miny;
    person_msg.width = color_width;
    person_msg.height = color_height;

//    person_msg.depth = user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z;
    //计算人的深度
    float person_x,person_y,person_z;
    DepthPixel depthv = pdepth[depth_y * depthFrame.getWidth() + depth_x];
    openni::CoordinateConverter::convertDepthToWorld(mDepthStream, depth_x, depth_y, depthv, &person_x, &person_y, &person_z);


//    static float last_depth = 0;
//    float torso_depth = user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z;
//
//    //cout<<"人的距离:  "<<torso_depth<<endl;
//
//    if(last_depth == torso_depth || torso_depth == 0)
//        person_msg.depth = 1800;
//    else
//        person_msg.depth = torso_depth;
//
//    last_depth = torso_depth;
//    cout<<"人的距离:  "<<min_depth<<endl;

    if(min_depth == 10000)
        person_msg.depth = 1800;
    else
        person_msg.depth =min_depth;

    chatter_pub->publish(person_msg);



    return true;
}


bool SampleViewer::localization_bag(nite::UserTracker *pUserTracker, const nite::UserData &user,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB, pcl::PointXYZ& hp, pcl::PointXYZ& hv) {

    float fscale = 0.001;

    pcl::PointXY p1_2d;             //手肘
    pcl::PointXY p2_2d;             //手腕

    pcl::PointXYZ p1_3d;             //手肘
    pcl::PointXYZ p2_3d;             //手腕

    p1_3d.x = tmpp1.x*fscale ;
    p1_3d.y = tmpp1.y*fscale;
    p1_3d.z = tmpp1.z*fscale;

    p2_3d.x = tmpp2.x*fscale;
    p2_3d.y = tmpp2.y*fscale;
    p2_3d.z = tmpp2.z*fscale;
    p1_2d=tmpp3;p2_2d=tmpp4;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_filtered->width = 640;
    cloud_filtered->height = 480;
    cloud_filtered->points.resize(cloud_XYZRGB->width * cloud_XYZRGB->height);

    pcl_seg_obj(cloud_XYZRGB, cloud_filtered,p1_2d,p2_2d,p1_3d,p2_3d,hp, hv);

    return true;
}

bool sendflag = true;

void SampleViewer::SendFlag(bool init_tracking, bool start_follow)
{
    sendflag =false;
    kcf_follower::flag flagMsg;
    flagMsg.init_tracking = init_tracking;
    flagMsg.start_follow = start_follow;
    statePub->publish(flagMsg);

}




//xian shi PCL
//pcl::visualization::PCLVisualizer::Ptr m_pViewer(new pcl::visualization::PCLVisualizer("Viewer"));
int init_voice_flag = 0;

void SampleViewer::Display() {

    //ros::spinOnce();
    //jici
    // mColorStream.start();
    // mDepthStream.start();

    if(init_voice_flag<2 ){
//        cout<<"!!!!!!!!!!!    "<<init_voice_flag<<endl;
        kcf_follower::voice voiceFile;
        voiceFile.filename = "/home/xjp/turtlebot_ws/src/kcf_follower/voice/xiaoI.mp3";
        voicePub->publish(voiceFile);
        //ros::Duration(5).sleep();
        init_voice_flag++;
    }



    nite::UserTrackerFrameRef userTrackerFrame;
    openni::VideoFrameRef depthFrame;
    openni::VideoFrameRef colorFrame;



    nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);
    if (rc != nite::STATUS_OK) {
        printf("GetNextData failed\n");
        return;
    }
    openni::DeviceInfo deviceInfo;

    mDepthStream.readFrame(&depthFrame);
    mColorStream.readFrame(&colorFrame);


    cv::Mat cImageBGR;
    cv::Mat dImageBGR;


    const cv::Mat mImageRGB(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3,
                            (void *) colorFrame.getData());  //color

    const cv::Mat dImageRGB(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1,
                            (void *) depthFrame.getData()); //depth


    cv::Mat mScaledDepth;

    dImageRGB.convertTo(mScaledDepth, CV_8U, 255.0 / MAX_DEPTH);


    cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);                                            //获取到彩色图cImageBGR
    cv::cvtColor(mScaledDepth, dImageBGR, CV_GRAY2BGR);                                    //获取到深度图dImageBGR


    sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImageBGR).toImageMsg();
    color_pub.publish(color_msg);

    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dImageBGR).toImageMsg();
    depth_pub.publish(depth_msg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_XYZRGB->width = 640;
    cloud_XYZRGB->height = 480;
    cloud_XYZRGB->points.resize(cloud_XYZRGB->width * cloud_XYZRGB->height);
    getCloudXYZCoordinate(cloud_XYZRGB, depthFrame);

    static int pcl_flag = 0;
    if (pcl_flag == 0) {
        cloud_viewer.runOnVisualizationThreadOnce (viewerOneOff);
        pcl_flag=1;
    }
//    m_pViewer->updatePointCloud<pcl::PointXYZRGB>(cloud_XYZRGB, "cloud");
//    m_pViewer->spinOnce();

    //获取深度数据
//    const openni::DepthPixel *pdepth = (DepthPixel*)depthFrame.getData();
//    cout<<" depth:"<<pdepth[240*depthFrame.getWidth()+320]<<endl;

//    int dX = 300;
//    int dY = 300;
//
//    int dX2 = 100;
//    int dY2 = 100;
//
//    int cX = 0;
//    int cY = 0;
//
//    int cX2 = 0;
//    int cY2 = 0;
//
//
//
//
//    DepthPixel depthv = pdepth[dY*depthFrame.getWidth() + dX];
//    DepthPixel depthv2 = pdepth[dY2*depthFrame.getWidth() + dX2];
//
//    openni::CoordinateConverter::convertDepthToColor(mDepthStream,mColorStream,dX,dY,depthv,&cX,&cY);
//    openni::CoordinateConverter::convertDepthToColor(mDepthStream,mColorStream,dX2,dY2,depthv,&cX2,&cY2);
//
//    cout<<"  cX:"<<cX<<"  cY:"<<cY<<"  depthv:"<<depthv<<"  cX2:"<<cX2<<"  cY2:"<<cY2<<"  depthv2:"<<depthv2<<endl;
//
//
//    cv::Point dp0 = cv::Point(dX,dY);
//    cv::Point dp1 = cv::Point(dX2, dY2);
//    cv::line(dImageBGR, dp0, dp1, cv::Scalar(0, 0, 255), 3, 4);
//
//    cv::Point cp0 = cv::Point(cX,cY);
//    cv::Point cp1 = cv::Point(cX2, cY2);
//    cv::line(cImageBGR, cp0, cp1, cv::Scalar(0, 0, 255), 3, 4);


//    cv::Mat dst;
//    resize(cImageBGR, dst, cv::Size(960, 520));
//    cv::imshow("color Image", dst);
//    cv::waitKey(1);
//    cv::imshow("init depth Image", dImageBGR);
//    cv::waitKey(1);




//显示
/*
	cv::namedWindow( "User Image",  CV_WINDOW_AUTOSIZE );
	while( true )
	{
        cv::imshow( "User Image", dImageBGR );
        if( cv::waitKey(1) == 'q')
            break;
	}

*/




    if (m_pTexMap == NULL) {
        // Texture map init
        m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
        m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
        m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
    }

    const nite::UserMap &userLabels = userTrackerFrame.getUserMap();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

    if (depthFrame.isValid() && g_drawDepth) {
        calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
    }

    memset(m_pTexMap, 0, m_nTexMapX * m_nTexMapY * sizeof(openni::RGB888Pixel));

    float factor[3] = {1, 1, 1};
    // check if we need to draw depth frame to texture
    if (depthFrame.isValid() && g_drawDepth) {
        const nite::UserId *pLabels = userLabels.getPixels();

        const openni::DepthPixel *pDepthRow = (const openni::DepthPixel *) depthFrame.getData();
        openni::RGB888Pixel *pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
        int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

        for (int y = 0; y < depthFrame.getHeight(); ++y) {
            const openni::DepthPixel *pDepth = pDepthRow;
            openni::RGB888Pixel *pTex = pTexRow + depthFrame.getCropOriginX();

            for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels) {
                if (*pDepth != 0) {
                    if (*pLabels == 0) {
                        if (!g_drawBackground) {
                            factor[0] = factor[1] = factor[2] = 0;

                        } else {
                            factor[0] = Colors[colorCount][0];
                            factor[1] = Colors[colorCount][1];
                            factor[2] = Colors[colorCount][2];
                        }
                    } else {
                        factor[0] = Colors[*pLabels % colorCount][0];
                        factor[1] = Colors[*pLabels % colorCount][1];
                        factor[2] = Colors[*pLabels % colorCount][2];
                    }
//					// Add debug lines - every 10cm
// 					else if ((*pDepth / 10) % 10 == 0)
// 					{
// 						factor[0] = factor[2] = 0;
// 					}

                    int nHistValue = m_pDepthHist[*pDepth];
                    pTex->r = nHistValue * factor[0];
                    pTex->g = nHistValue * factor[1];
                    pTex->b = nHistValue * factor[2];

                    factor[0] = factor[1] = factor[2] = 1;
                }
            }

            pDepthRow += rowSize;
            pTexRow += m_nTexMapX;
        }
    }

    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

    // Display the OpenGL texture map
    glColor4f(1, 1, 1, 1);

    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);

    g_nXRes = depthFrame.getVideoMode().getResolutionX();
    g_nYRes = depthFrame.getVideoMode().getResolutionY();

//    int c_nXRes = colorFrame.getVideoMode().getResolutionX();
//    int c_nYRes = colorFrame.getVideoMode().getResolutionY();

    //   cout<<"  X     "<<c_nXRes<<"    Y     "<<c_nYRes<<endl;
    // upper left
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    // upper right
    glTexCoord2f((float) g_nXRes / (float) m_nTexMapX, 0);
    glVertex2f(GL_WIN_SIZE_X, 0);
    // bottom right
    glTexCoord2f((float) g_nXRes / (float) m_nTexMapX, (float) g_nYRes / (float) m_nTexMapY);
    glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    // bottom left
    glTexCoord2f(0, (float) g_nYRes / (float) m_nTexMapY);
    glVertex2f(0, GL_WIN_SIZE_Y);

    glEnd();
    glDisable(GL_TEXTURE_2D);

    const nite::Array<nite::UserData> &users = userTrackerFrame.getUsers();
    for (int i = 0; i < users.getSize(); ++i) {
        const nite::UserData &user = users[i];

        updateUserState(user, userTrackerFrame.getTimestamp());
        if (user.isNew()) {
            m_pUserTracker->startSkeletonTracking(user.getId());
            m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
        } else if (!user.isLost()) {
            if (g_drawStatusLabel) {
                DrawStatusLabel(m_pUserTracker, user);
            }
            if (g_drawCenterOfMass) {
                DrawCenterOfMass(m_pUserTracker, user);
            }
            if (g_drawBoundingBox) {
                DrawBoundingBox(user);
            }

            if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawSkeleton) {
                DrawSkeleton(m_pUserTracker, user);

                //从这里开始是我写的
                //收拾判别，根据右臂的大臂和小臂的夹角余弦来判断，如果接近1说明两者同向，规定为指派物品
                //如果接近于0，则规定为开始命令，要求机器人跟随


                int now_ID = user.getId();
                //cout<<"now  ID  is   "<< now_ID<<endl;


                //右手
                float rightHandX = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x;
                float rightHandY = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y;
                float rightHandZ = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().z;
                float rightElbowX = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().x;
                float rightElbowY = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y;
                float rightElbowZ = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().z;
                float rightShoulderX = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().x;
                float rightShoulderY = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().y;
                float rightShoulderZ = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().z;
                
                right_vec1.x = rightElbowX - rightShoulderX;
                right_vec1.y = rightElbowY - rightShoulderY;
                right_vec1.z = rightElbowZ - rightShoulderZ;
                right_vec2.x = rightHandX - rightElbowX;
                right_vec2.y = rightHandY - rightElbowY;
                right_vec2.z = rightHandZ - rightElbowZ;
                float right_v1xv2 =
                        right_vec1.x * right_vec2.x + right_vec1.y * right_vec2.y + right_vec1.z * right_vec2.z;
                float right_mod1 = sqrt(
                        right_vec1.x * right_vec1.x + right_vec1.y * right_vec1.y + right_vec1.z * right_vec1.z);
                float right_mod2 = sqrt(
                        right_vec2.x * right_vec2.x + right_vec2.y * right_vec2.y + right_vec2.z * right_vec2.z);
                float cos = fabs(right_v1xv2 / (right_mod1 * right_mod2));


                //到这里为止，前面的程序是我写的

                //左手
                float leftHandX = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x;
                float leftHandY = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y;
                float leftHandZ = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().z;
                float leftElbowX = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().x;
                float leftElbowY = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y;
                float leftElbowZ = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().z;



                //cout<<"leftElboxX :    "<<leftElbowX <<"      leftElbowY :"<<leftElbowY<<"     leftElbowZ  : "  <<leftElbowZ<<endl;



                float leftShoulderX = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().x;
                float leftShoulderY = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().y;
                float leftShoulderZ = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().z;

                left_vec1.x = leftElbowX - leftShoulderX;
                left_vec1.y = leftElbowY - leftShoulderY;
                left_vec1.z = leftElbowZ - leftShoulderZ;
                left_vec2.x = leftHandX - leftElbowX;
                left_vec2.y = leftHandY - leftElbowY;
                left_vec2.z = leftHandZ - leftElbowZ;
                float left_v1xv2 = left_vec1.x * left_vec2.x + left_vec1.y * left_vec2.y + left_vec1.z * left_vec2.z;
                float left_mod1 = sqrt(
                        left_vec1.x * left_vec1.x + left_vec1.y * left_vec1.y + left_vec1.z * left_vec1.z);
                float left_mod2 = sqrt(
                        left_vec2.x * left_vec2.x + left_vec2.y * left_vec2.y + left_vec2.z * left_vec2.z);
                float left_cos = fabs(left_v1xv2 / (left_mod1 * left_mod2));

                float left_angle = left_vec1.y / left_mod1;
                float right_angle = right_vec1.y / right_mod1;


                if (left_angle < 0.86 || right_angle < 0.86)                        //有手抬起
                {

                    if (left_angle < right_angle) {
                        //cout<<"LEFT hand"<<endl;					//左手抬起
                        if (order != 1)
                            orderCnt = 0;
                        else
                            orderCnt++;
                        order = 1;

                    } else {
                        //cout<<"RIGHT hand"<<endl;				//右手抬起
                        if (order != 0)
                            orderCnt = 0;
                        else
                            orderCnt++;
                        order = 0;
                    }
                }

               // cout << " 点云测试" << endl;
                //if dian yun ji suan !
                static int delay_count = 0;
                if(cloud_point == true  && delay_count ==0) {
                    kcf_follower::voice voiceFile;
                    voiceFile.filename = "/home/xjp/turtlebot_ws/src/kcf_follower/voice/readyForBag.mp3";
                    voicePub->publish(voiceFile);
                    //        sleep(10);
                    delay_count++;
                }
                if(delay_count>0 && delay_count++>100){
                    cout<<"  PCL  PCL  PCL   PCL   PCL!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                    //点云测试
                    tmpp1.x=rightElbowX;tmpp1.y=rightElbowY;tmpp1.z=rightElbowZ;
                    tmpp2.x=rightHandX;tmpp2.y=rightHandY;tmpp2.z=rightHandZ;
                    m_pUserTracker->convertJointCoordinatesToDepth(tmpp2.x, tmpp2.y, tmpp2.z,
                                                                   &tmpp4.x, &tmpp4.y);

                    m_pUserTracker->convertJointCoordinatesToDepth(tmpp1.x, tmpp1.y, tmpp1.z,
                                                                   &tmpp3.x, &tmpp3.y);

                    localization_bag(m_pUserTracker, user, cloud_XYZRGB, hole_point, hole_vec);
                    cloud_viewer.showCloud(cloud_XYZRGB);
                    cloud_viewer.runOnVisualizationThreadOnce(show_hole);
                    cloud_viewer.wasStopped();
                    if(hole_point.z == 0){
                        //no bag here
                        kcf_follower::voice voiceFile;
                        voiceFile.filename = "/home/xjp/turtlebot_ws/src/kcf_follower/voice/noBag.mp3";
                        voicePub->publish(voiceFile);
                        delay_count=0;
                    }
                    else{
                        //find bag, and never come in again
                        delay_count=-1;
                        ros::Duration(5).sleep();
                        kcf_follower::voice bag_voiceFile;
                        bag_voiceFile.filename = "/home/xjp/turtlebot_ws/src/kcf_follower/voice/getBag.mp3";
                        voicePub->publish(bag_voiceFile);
                        now_is_goodbye = true;
                    }
                }

//                float fscale = 0.001;
//
//                //depth image test  start
//                pcl::PointXY p1_2d;             //手肘
//                pcl::PointXY p2_2d;             //手腕
//                m_pUserTracker->convertJointCoordinatesToDepth(rightHandX, rightHandY, rightHandZ,
//                                                               &p2_2d.x, &p2_2d.y);
//
//                m_pUserTracker->convertJointCoordinatesToDepth(rightElbowX, rightElbowY, rightElbowZ,
//                                                               &p1_2d.x, &p1_2d.y);
//
//                //画线的坐标，起始坐标和终止坐标
//
//                cv::Point cv_p0 = cv::Point((int) p2_2d.x, (int) p2_2d.y);
//                cv::Point cv_p1 = cv::Point((int) p1_2d.x, (int) p1_2d.y);
//                cv::line(dImageBGR, cv_p0, cv_p1, cv::Scalar(0, 0, 255), 3, 4);
//
//
//
//                int i = (int) p2_2d.x;
//                int j = (int) p2_2d.y;
//                i = limit_x(i);
//                j = limit_y(j);
//                const openni::DepthPixel *pdepth = (DepthPixel*)depthFrame.getData();
//                DepthPixel depthv = pdepth[j*depthFrame.getWidth() + i];
//
//                float hand_x, hand_y, hand_z;
//                openni::CoordinateConverter::convertDepthToWorld(mDepthStream, (int) p2_2d.x, (int) p2_2d.y, depthv, &hand_x, &hand_y, &hand_z);
//
//                //证明OpenNI2里的世界坐标系和NITE库的关节坐标系是一个东西
//                cout<<"hand_x:"<<hand_x<<"  rightHandX:"<<rightHandX<<"  hand_y:"<<hand_y<<"  rightHandY:"<<rightHandY<<"  hand_z: "<<hand_z<<"  rightHandZ: "<<rightHandZ<<endl;
//
//                //显示
//                cv::imshow("depth Image", dImageBGR);
//                cv::waitKey(1);



//              m_pViewer->addCoordinateSystem(0.3);
//              pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_XYZRGB);
//              m_pViewer->removeAllPointClouds();
//              m_pViewer->spinOnce();


                //发送消息出去
                gestures_extract::body msg;
                msg.x = user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().x;
                msg.y = user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().y;
                msg.z = user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z;


 

            //    chatter_pub->publish(msg);
            }
           
           SendMsg(user, color_msg, 0, depthFrame);              //发送信息给家鹏

            //color_img测试
           const openni::DepthPixel *pdepth = (DepthPixel*)depthFrame.getData();
           int depth_minx = user.getBoundingBox().min.x;
           int depth_miny = user.getBoundingBox().min.y;
           int depth_maxx = user.getBoundingBox().max.x;
           int depth_maxy = user.getBoundingBox().max.y;

           depth_minx = limit_x(depth_minx);
           depth_maxx=limit_x(depth_maxx);

           depth_miny = limit_y(depth_miny);
           depth_maxy=limit_y(depth_maxy);

           int color_minx = 0;
           int color_miny = 0;
           int color_maxx = 0;
           int color_maxy = 0;

           DepthPixel depthvmin = pdepth[depth_miny*depthFrame.getWidth() + depth_minx];
           DepthPixel depthvmax = pdepth[depth_maxy*depthFrame.getWidth() + depth_maxx];

           openni::CoordinateConverter::convertDepthToColor(mDepthStream,mColorStream,depth_minx,depth_miny,1500,&color_minx,&color_miny);
           openni::CoordinateConverter::convertDepthToColor(mDepthStream,mColorStream,depth_maxx,depth_maxy,1500,&color_maxx,&color_maxy);



           int color_width = color_maxx - color_minx;
           int color_height = color_maxy - color_miny;



//           cout<<"  color_minx"<<color_minx<<" color_miny"<<color_miny<<"  color_width"<<color_width<<"  color_height"<<color_height<<endl;

           cv::Rect rect(color_minx, color_miny, color_width, color_height);//左上坐标（x,y）和矩形的长(x)宽(y)

           cv::rectangle(cImageBGR, rect, cv::Scalar(0, 255, 0),2, cv::LINE_8,0);
           cv::Mat dst;
           resize(cImageBGR, dst, cv::Size(960, 540));
           cv::imshow("color Image ", dst);
           cv::waitKey(1);




        }

        // if (m_poseUser == 0 || m_poseUser == user.getId()) {
        //     const nite::PoseData &pose = user.getPose(nite::POSE_CROSSED_HANDS);

        //     if (pose.isEntered()) {
        //         // Start timer
        //         sprintf(g_generalMessage, "In exit pose. Keep it for %d second%s to exit\n", g_poseTimeoutToExit / 1000,
        //                 g_poseTimeoutToExit / 1000 == 1 ? "" : "s");
        //         printf("Counting down %d second to exit\n", g_poseTimeoutToExit / 1000);
        //         m_poseUser = user.getId();
        //         m_poseTime = userTrackerFrame.getTimestamp();
        //     } else if (pose.isExited()) {
        //         memset(g_generalMessage, 0, sizeof(g_generalMessage));
        //         printf("Count-down interrupted\n");
        //         m_poseTime = 0;
        //         m_poseUser = 0;
        //     } else if (pose.isHeld()) {
        //         // tick
        //         if (userTrackerFrame.getTimestamp() - m_poseTime > g_poseTimeoutToExit * 1000) {
        //             printf("Count down complete. Exit...\n");
        //             Finalize();
        //             exit(2);
        //         }
        //     }
        // }


    }


    if (orderCnt > 100) {
        if (order == 1) {
            cout << " now     state     is:    right" << endl;                    //右手
        }
        if (order == 0) {
            cout << " now     state     is:    left" << endl;                        //左手
        }

        order = -1;
        orderCnt = 0;
    }


    if (g_drawFrameId) {
        DrawFrameId(userTrackerFrame.getFrameIndex());
    }

    if (g_generalMessage[0] != '\0') {
        char *msg = g_generalMessage;
        glColor3f(1.0f, 0.0f, 0.0f);
        glRasterPos2i(100, 20);
        glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
    }



    // Swap the OpenGL display buffers
    glutSwapBuffers();


    /*
    std_msgs::String msg;            //申请内存
    std::stringstream ss;                  //信息内容变量
    ss << "I am publisher ";         //写入变量
    msg.data = ss.str();             //写入内存

*/
        // 读取帧信息

    sendflag = true;
    //POSE_PSI ->  tracking
    //GESTURE_WAVE -> init
    //GESTURE_CLICK -> cloud_PCL

    HandTrackerFrameRef mHandFrame;
    mHandTracker.readFrame( &mHandFrame );

    // 整个界面帧信息进行分析，找到符合的手势
    const nite::Array<GestureData>& aGestures = mHandFrame.getGestures();

    for( int i = 0; i < aGestures.getSize(); ++ i )
    {
        const GestureData& rGesture = aGestures[i];

        // 对找到的手势进行类型判断，并输出类型
       // cout << "Detect gesture ";
        switch( rGesture.getType() )
        {
        case GESTURE_WAVE:
            //cout << "摇手手势---“wave”：";
//            cout << "init_tracking!!!!!!! "<<endl;
//            init_tracking = true;
//            start_tracking = false;
//            cloud_point = false;
        //    SendFlag(true,false);
            break;

//        case GESTURE_CLICK:
        //    cout << "前推并收回手势---“click”";
//            cout<<"cloud_PCL!!!!!!"<<endl;
//            init_tracking = false;
//            start_tracking = false;
//            cloud_point = true;
            //

            //
        //    SendFlag(false,false);
 //           break;

//         case GESTURE_HAND_RAISE:
//             cout << "init_tracking!!!!!!! "<<endl;
//             init_tracking = true;
//             start_tracking = false;
//             cloud_point = false;
//             //cout << "举起手势---“hand raise”";
//             break;
        }

        // 得到的手势信息中还包含了当前手势的坐标位置
//        const Point3f& rPos = rGesture.getCurrentPosition();
//        cout << " 手势位置为： （" << rPos.x << ", " << rPos.y << ", " << rPos.z << ")" << endl;

        // 以及手势状态，完成状态和进行状态
        if( rGesture.isComplete() )
        {
        //    SendFlag(false,false);
        //    cout << "  手势完成"<<endl;
        }

        if( rGesture.isInProgress() ) {
        //    SendFlag(false, false);
        //    cout << "  手势正在进行" << endl;
        }
    }

    // 通过帧信息，获得用户数据UserData
    const nite::Array<UserData>& aUsers = userTrackerFrame.getUsers();

    for( int i = 0; i < aUsers.getSize(); ++ i )
    {
        const UserData& rUser = aUsers[i];
        const UserId& uID = rUser.getId();

        if( rUser.isNew() )
        {
        //    cout << "User " << uID << " found." << endl;

            // 为每一个新用户进行姿势探测
        //    cout << "Start pose detection " << uID<< endl;

            // 探测uID的两种姿势
            m_pUserTracker->startPoseDetection( uID, POSE_PSI );
            m_pUserTracker->startPoseDetection( uID, POSE_CROSSED_HANDS );
        }
        else if( rUser.isLost() )
        {
        //    cout << "User " << uID << " lost." << endl;
        }
        else
        {
            // 读取用户的“POSI_PSI”的姿势状态
            const PoseData& rPosePSI = rUser.getPose( POSE_PSI );

            // 当做了“POSI_PSI”时：
            if( rPosePSI.isEntered() ) {
            //    SendFlag(false, true);
                cout<<"start_follow!!!!!!"<<endl;
                init_tracking = false;
                start_tracking = true;
                cloud_point = false;
            //    cout << " 开始---投降姿势(PSI pose)" << endl;
            }

            if( rPosePSI.isHeld() )
            {
            //    SendFlag(false, false);
            //    cout << " 保持---投降姿势(PSI pose)" << endl;
            }


            // 当做完了“POSI_PSI”后，双手放下时：
            if( rPosePSI.isExited() )
            {
            //    SendFlag(false, false);
            //    cout << " 停止---投降姿势(PSI pose)" << endl;
            }

            // 同样的读取“POSE_CROSSED_HANDS”的姿势状态
            const PoseData& rPCH = rUser.getPose( POSE_CROSSED_HANDS );

            if( rPCH.isEntered() )
            {
                cout<<"here1    RISE  RISE"<<endl;
                if(now_is_goodbye == false) {
                    cout << "cloud_PCL!!!!!!" << endl;
                    init_tracking = false;
                    start_tracking = false;
                    cloud_point = true;
                }
                else
                {
                    cout << "good_bye!!!!!!! "<<endl;
                    init_tracking = true;
                    start_tracking = false;
                    cloud_point = false;
                }
            }
            //    cout << " 开始---双手抱胸(Cross Hand pose)" << endl;

        //    if( rPCH.isHeld() )
            //    cout << " 保持---双手抱胸(Cross Hand pose)" << endl;

        //    if( rPCH.isExited() )
            //    cout << " 停止---双手抱胸(Cross Hand pose)" << endl;
        }
    }

    if(init_tracking)
        SendFlag(true,false);
    else if(start_tracking)
    {
        SendFlag(false,true);
    }
    else{
        SendFlag(false,false);
    }
    


}

void SampleViewer::OnKey(unsigned char key, int /*x*/, int /*y*/) {
    switch (key) {
        case 27:
            Finalize();
            exit(1);
        case 's':
            // Draw skeleton?
            g_drawSkeleton = !g_drawSkeleton;
            break;
        case 'l':
            // Draw user status label?
            g_drawStatusLabel = !g_drawStatusLabel;
            break;
        case 'c':
            // Draw center of mass?
            g_drawCenterOfMass = !g_drawCenterOfMass;
            break;
        case 'x':
            // Draw bounding box?
            g_drawBoundingBox = !g_drawBoundingBox;
            break;
        case 'b':
            // Draw background?
            g_drawBackground = !g_drawBackground;
            break;
        case 'd':
            // Draw depth?
            g_drawDepth = !g_drawDepth;
            break;
        case 'f':
            // Draw frame ID
            g_drawFrameId = !g_drawFrameId;
            break;
    }

}

openni::Status SampleViewer::InitOpenGL(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow(m_strSampleName);
    // 	glutFullScreen();
    glutSetCursor(GLUT_CURSOR_NONE);

    InitOpenGLHooks();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);


    return openni::STATUS_OK;

}

void SampleViewer::InitOpenGLHooks() {
    glutKeyboardFunc(glutKeyboard);
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);
}
