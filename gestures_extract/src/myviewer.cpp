//
// Created by isness on 2020/10/9.
//
#include <mutex>
#include <thread>
#include <chrono>

#include "std_msgs/String.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ros/spinner.h>
#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>

#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension
#include <pcl/common/transforms.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cv_bridge/cv_bridge.h>
//#include <kinect2_bridge/kinect2_definitions.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


static void pcl_filter_seg_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);

static void visual_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clouds);

void process_input(int argc, char **pString);

int num = 0;

void process_input(int argc, char **pString) {
    if (argc > 1)num = std::stoi(pString[1]);
    else num = 50;
}

void somecode() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    /*filter: passthrough*/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    /*down sampling*/
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    /*statistical removal*/
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
    sor1.setInputCloud(cloud);
    sor1.setMeanK(50);
    sor1.setStddevMulThresh(1.0);
    sor1.filter(*cloud_filtered);

    /*plane segmentation*/
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    // Create the filtering object

    int i = 0, nr_points = (int) cloud_filtered->size();
    // While 30% of the original cloud is still there
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(
            new pcl::PointCloud<pcl::PointXYZ>);
    while (cloud_filtered->size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
                  << " data points." << std::endl;
        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }

    /*region growth segmentation*/
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster;
    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_cluster);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud_cluster);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    /*I/O*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud1) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    pcl::PCDReader reader;
    reader.read("/home/isness/pcd files/0000_cloud.pcd", *cloud);


    /*radius outlier removal*/
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
    //build the filter
    outrem.setInputCloud(cloud_cluster);
    outrem.setRadiusSearch(0.1);
    outrem.setMinNeighborsInRadius(num);
//    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*cloud_cluster);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZRGBA> pass1;
    pass1.setInputCloud(cloud_cluster);
    pass1.setFilterFieldName("z");
    pass1.setFilterLimits(0.0, 1.0);
    pass1.filter(*indices);

    /*visualizer */
    pcl::visualization::CloudViewer viewer("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped()) {}


};
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clouds;


/*
 * some old code
 */
//static void pcl_filter_seg_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
//                                 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered) {
//    pcl::PassThrough<pcl::PointXYZRGBA> pass;
//    /*distance filter*/
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_dis_filter(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pass.setInputCloud(cloud);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0.0, 1);
//    //pass.setFilterLimitsNegative (true);
//    pass.filter(*cloud_dis_filter);
//
//
//    /*plane filter*/
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane_filter(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
//    seg.setOptimizeCoefficients(true);
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(1000);
//    seg.setDistanceThreshold(0.02);
//    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f(
//            new pcl::PointCloud<pcl::PointXYZRGBA>);
//    seg.setInputCloud(cloud_dis_filter);
//    seg.segment(*inliers, *coefficients);
//    if (inliers->indices.size() < cloud_dis_filter->size() * 0.4) {
//        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//    } else {
//        extract.setInputCloud(cloud_dis_filter);
//        extract.setIndices(inliers);
//        extract.setNegative(false);
//        extract.filter(*cloud_p);
//        printf("raw cloud %d points, %dx%d\n", cloud_dis_filter->size(), cloud_dis_filter->width,
//               cloud_dis_filter->height);
//        printf("point cloud get plane %d points, %dx%d\n", cloud_p->size(), cloud_p->width, cloud_p->height);
//        std::cerr << "the coefficients: a(" << coefficients->values[0] << "), b(" << coefficients->values[1] << "), c("
//                  << coefficients->values[2] << "), z(" << coefficients->values[3] << ")\n";
//        extract.setNegative(true);
//        extract.filter(*cloud_f);
//    }
//    seg.setDistanceThreshold(0.005);
//    seg.setInputCloud(cloud_f);
//    seg.segment(*inliers, *coefficients);
//    if (inliers->indices.size() < cloud_f->size() * 0.15) {
//        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//        cloud_f.swap(cloud);
//    } else {
//        extract.setInputCloud(cloud_f);
//        extract.setIndices(inliers);
//        extract.setNegative(false);
//        extract.filter(*cloud_p);
//        printf("raw cloud %d points, %dx%d\n", cloud_f->size(), cloud_f->width, cloud_f->height);
//        printf("point cloud get plane %d points, %dx%d\n", cloud_p->size(), cloud_p->width, cloud_p->height);
//        std::cerr << "the coefficients: a(" << coefficients->values[0] << "), b(" << coefficients->values[1] << "), c("
//                  << coefficients->values[2] << "), z(" << coefficients->values[3] << ")\n";
//        extract.setNegative(true);
//        extract.filter(*cloud_f);
//        cloud_p->swap(*cloud);
//    }
//
//
//    /*statistical points filter*/
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor1;
//    sor1.setInputCloud(cloud);
//    sor1.setMeanK(100);
//    sor1.setStddevMulThresh(0.2);
//    sor1.filter(*cloud_filtered);
//
//    /*bound extractor*/
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    pcl::PointCloud<pcl::Boundary> boundaries;
//    pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
//    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
//    //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
//    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
//    kdtree.setInputCloud(cloud_filtered);
//    int k = 2;
//    float everagedistance = 0;
//    for (int i = 0; i < cloud_filtered->size() / 2; i++) {
//        //std::cout << "cloud->size()/2" << cloud->points[i] << std::endl;
//        std::vector<int> nnh;
//        std::vector<float> squaredistance;
//        //  pcl::PointXYZ p;
//        //   p = cloud->points[i];
//        kdtree.nearestKSearch(cloud_filtered->points[i], k, nnh, squaredistance);
//        /*std::cout << "查询点位： " << cloud->points[i] << std::endl;
//        std::cout << "近邻为： " << nnh[0] << "  " << nnh[1] << std::endl;
//        std::cout << "近邻为： " << cloud->points[nnh[0]] << "  " << cloud->points[nnh[1]] << std::endl;
//*/
//        everagedistance += sqrt(squaredistance[1]);
//        //   cout<<everagedistance<<endl;
//
//    }
//    everagedistance = everagedistance / (cloud_filtered->size() / 2);
//    cout << "everage distance is : " << everagedistance << endl;
//
//    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
//    normEst.setInputCloud(cloud_filtered);
//    normEst.setSearchMethod(tree);
//    // normEst.setRadiusSearch(2);  //法向估计的半径
//    normEst.setKSearch(9);  //法向估计的点数
//    normEst.compute(*normals);
//    cout << "normal size is " << normals->size() << endl;
//
//    //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
//    est.setInputCloud(cloud_filtered);
//    est.setInputNormals(normals);/*M_PI_2 */
//    est.setAngleThreshold(M_PI_2);   ///在这里 由于构造函数已经对其进行了初始化 为Π/2 ，必须这样 使用 M_PI/2  M_PI_2
//    est.setSearchMethod(tree);
//    est.setKSearch(120);  //一般这里的数值越高，最终边界识别的精度越好 20
//    //  est.setRadiusSearch(everagedistance);  //搜索半径
//    est.compute(boundaries);
//    int num = 0;
//    for (int i = 0; i < cloud_filtered->points.size(); i++) {
//        if (boundaries.points[i].boundary_point > 0) {
//            cloud_filtered->points[i].r = 255;
//            cloud_filtered->points[i].g = 255;
//            cloud_filtered->points[i].b = 255;
//            num++;
//        }
//    }
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
//    ec.setClusterTolerance(0.02); // 2cm
//    ec.setMinClusterSize(100);
//    ec.setMaxClusterSize(25000);
//    ec.setSearchMethod(tree);
//    ec.setInputCloud(cloud_filtered);
//    ec.extract(cluster_indices);
//
//    printf("the num of boundary%d\n", num);
//    /*range image*/
////    float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
////    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
////    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
////    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
////    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
////    float noiseLevel=0.00;
////    float minRange = 0.0f;
////    int borderSize = 1;
////
////    pcl::RangeImage range_image;
////    range_image.createFromPointCloud(*cloud_filtered, angularResolution, maxAngleWidth, maxAngleHeight,
////                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//
//    // --------------------------------------------
//    // -----Open 3D viewer and add point cloud-----
//    // --------------------------------------------
////    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
////    viewer.setBackgroundColor (1, 1, 1);
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> point_cloud_color_handler (cloud_filtered, 0, 0, 0);
////    viewer.addPointCloud (cloud_filtered, point_cloud_color_handler, "original point cloud");
////
////    pcl::RangeImageBorderExtractor border_extractor (&range_image);
////    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
////    border_extractor.compute (border_descriptions);
////    for(int i=0; i<border_descriptions.points.size(); i++){
////        border_descriptions.points[i];
//////        cloud_filtered->points[->r=255;
//////        cloud_filtered->points.data()->g=255;
//////        cloud_filtered->points.data()->b=255;
////    }
//    // ----------------------------------
//    // -----Show points in 3D viewer-----
//    // ----------------------------------
//    /*pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
//            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
//            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
//    pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
//            & veil_points = * veil_points_ptr,
//            & shadow_points = *shadow_points_ptr;
//    for (int y=0; y< (int)range_image.height; ++y)
//    {
//        for (int x=0; x< (int)range_image.width; ++x)
//        {
//            if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
//                border_points.points.push_back (range_image[y*range_image.width + x]);
//            if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
//                veil_points.points.push_back (range_image[y*range_image.width + x]);
//            if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
//                shadow_points.points.push_back (range_image[y*range_image.width + x]);
//        }
//    }
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
//    viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
//    viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
//    viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
//
//    //-------------------------------------
//    // -----Show points on range image-----
//    // ------------------------------------
//    pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
//    range_image_borders_widget =
//            pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
//                                                                                  border_descriptions, "Range image with borders");
//    // -------------------------------------
//
//
//    //--------------------
//    // -----Main loop-----
//    //--------------------
//    while (!viewer.wasStopped ())
//    {
//        range_image_borders_widget->spinOnce ();
//        viewer.spinOnce ();
//        pcl_sleep(0.01);
//    }*/
//
////    clouds.push_back(cloud);
////    clouds.push_back(cloud_filtered);
////    clouds.push_back(cloud_f);
////    clouds.push_back(cloud_p);
//    /*visualize the points cloud*/
////    visual_clouds(clouds);
////    pcl::visualization::CloudViewer viewer("cloud"),viewer1("cloud_filtered"),viewer2("cloud_p"),viewer3("cloud_f");
////    viewer.showCloud(cloud);
////    viewer1.showCloud(cloud_filtered);
////    viewer2.showCloud(cloud_p);
////    viewer3.showCloud(cloud_f);
//
////    while (!viewer.wasStopped());//&&!viewer1.wasStopped()&&!viewer2.wasStopped()&&!viewer3.wasStopped()) {}
//
//}


double DistanceOfPointToLine(pcl::PointXYZ *a, pcl::PointXYZ *b, pcl::PointXYZ *s) {
    double ab = sqrt(pow((a->x - b->x), 2.0) + pow((a->y - b->y), 2.0) + pow((a->z - b->z), 2.0));
    double as = sqrt(pow((a->x - s->x), 2.0) + pow((a->y - s->y), 2.0) + pow((a->z - s->z), 2.0));
    double bs = sqrt(pow((s->x - b->x), 2.0) + pow((s->y - b->y), 2.0) + pow((s->z - b->z), 2.0));
    double cos_A = (as*as + ab*ab - bs*bs) / (2 * ab * as);
    double sin_A = sqrt(1 - cos_A*cos_A);
    return as * sin_A;
}
double distance(pcl::PointXYZRGBA&a,pcl::PointXYZRGBA&b ){
    double x=a.x-b.x, y=a.y-b.y, z=a.z-b.z;
    return sqrt(x*x+y*y+z*z);
}

double cosvec(const pcl::Normal& nor1, const pcl::Normal & nor2){
    float x1=nor1.normal_x,x2=nor2.normal_x,y1=nor1.normal_y,y2=nor2.normal_y,z1=nor1.normal_z,z2=nor2.normal_z;
    return (x1*x2+y1*y2+z1*z2)/sqrt(x1*x1+y1*y1+z1*z1)/sqrt(x2*x2+y2*y2+z2*z2);
}
static void pcl_seg_obj(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,
                //        pcl::PointXY p1_2d,
                 //       pcl::PointXY p2_2d,
                        pcl::visualization::PCLVisualizer::Ptr v1,
                        pcl::visualization::PCLVisualizer::Ptr v2) {

    int rows = cloud->height, cols = cloud->width;
    //pcl::PointXY p1_2d=pcl::PointXY(222, 136);
    //pcl::PointXY p2_2d=pcl::PointXY(241, 156);
    pcl::PointXY p1_2d;
    p1_2d.x = 222;
    p1_2d.y = 136;

    pcl::PointXY p2_2d;
    p2_2d.x = 241;
    p2_2d.y = 156;

    pcl::PointXYZ p1_3d(cloud->points[cols * p1_2d.y + p1_2d.x].x, cloud->points[cols * p1_2d.y + p1_2d.x].y,
                        cloud->points[cols * p1_2d.y + p1_2d.x].z);
    pcl::PointXYZ p2_3d(cloud->points[cols * p2_2d.y + p2_2d.x].x, cloud->points[cols * p2_2d.y + p2_2d.x].y,
                        cloud->points[cols * p2_2d.y + p2_2d.x].z);

    pcl::PointXYZ pv_3d(p2_3d.x - p1_3d.x, p2_3d.y - p1_3d.y, p2_3d.z - p1_3d.z);
//    printf("p1(%.2f,%.2f,%.2f),p2(%.2f,%.2f,%.2f)\n",p1_3d.x,p1_3d.y,p1_3d.z,p2_3d.x,p2_3d.y,p2_3d.z);
    float pv_dis = sqrt(pv_3d.x * pv_3d.x + pv_3d.y * pv_3d.y + pv_3d.z * pv_3d.z);
    pv_3d.x /= pv_dis;
    pv_3d.y /= pv_dis;
    pv_3d.z /= pv_dis;
    int scale_pix = 10;
    float radius = 0.2;
    float vec_pix_x = (p2_2d.x - p1_2d.x) / sqrt(pow(p2_2d.x - p1_2d.x, 2) + pow(p2_2d.y - p1_2d.y, 2));
    float vec_pix_y = (p2_2d.y - p1_2d.y) / sqrt(pow(p2_2d.x - p1_2d.x, 2) + pow(p2_2d.y - p1_2d.y, 2));
    int scan_cnt=0;
    int scan_thresh = 300;

    //索引出对应的包， 0为未判断，1是包，-1不是包
    int indices[rows*cols];
    //初始化数组为0，注意sizeof(int)
    memset(indices, 0, rows*cols*sizeof(int));
    std::list<int> indexPos;

    auto start = std::chrono::system_clock::now();
    //初步索引部分目标点
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (float x = p2_2d.x + 2*scale_pix*vec_pix_x / vec_pix_y, y = p2_2d.y+2*scale_pix;
         x < cols - scale_pix && y < rows && x > scale_pix && y > 0; x += vec_pix_x / vec_pix_y, y++) {
        for (int j =  - scale_pix, pos = (int) (y * cols + x - scale_pix); j < scale_pix; j++, pos++) {

            if(std::isnan(cloud->points[pos].x) ){
                indices[pos] = -1;
                continue;
            }

            pcl::PointXYZ tmp(cloud->points[pos].x, cloud->points[pos].y, cloud->points[pos].z);
            double dis = DistanceOfPointToLine(&p1_3d, &p2_3d, &tmp);

            if ( dis < radius) {
//                printf("p%d w%d h%d x%.2f y%.1f j%d dis%.2f cnt%d\n", pos, cols, rows, x, y,  j, dis, scan_cnt);
                scan_cnt++;
                indices[pos] = 1;
                inliers->indices.push_back(pos);
                indexPos.push_back(pos);
            } else {

            }
        }
        if(scan_cnt > scan_thresh) break;
    }
    auto end   = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "初步索引部分目标点用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();
    //法线估计
    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    int maxpos=rows*cols;
    float costh = cos(6.0/180*M_PI);
    float disth = 0.01;
    int edgeth=11;
    std::vector<int> edgePos;

    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "计算法线用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();
    //初步生长
    while( !indexPos.empty() ) {
        int ip = indexPos.front();
        indexPos.pop_front();
        if (ip > maxpos - cols - 2) continue;

        std::vector<int> nearPos;

        nearPos.push_back(ip - cols-1);
        nearPos.push_back(ip - cols);
        nearPos.push_back(ip - cols+1);
        nearPos.push_back(ip - 1);
        nearPos.push_back(ip + 1);
        nearPos.push_back(ip + cols-1);
        nearPos.push_back(ip + cols);
        nearPos.push_back(ip + cols+1);

        pcl::PointXYZRGBA &seed = cloud->points[ip];
        pcl::Normal &nor = normals->points[ip];
        int nearNum = 0;
        for (int i = 0; i < nearPos.size();i++) {
            int np = nearPos[i];
            pcl::PointXYZRGBA &seed1 = cloud->points[np];
            pcl::Normal & nor1 = normals->points[np];
            if(std::isnan(seed1.x)){
                indices[np] = -1;
                continue;
            }
            if (indices[np] == 0 ){
                double dis = distance(seed1, seed), cv=cosvec(nor, nor1);
                if(   dis < disth &&  abs(cv)> costh) {
                    nearNum++;
//                    printf("ip%d np%d dis%.2f cosvec%.2f costh%.2f\n",ip, np, dis, cv, costh );
                    indices[np] = 1;
                    inliers->indices.push_back(np);
                    indexPos.push_back(np);
                }else {
                    indices[np] = -1;
                }
            }else if(indices[np]==1) nearNum++;
        }
        if(nearNum < edgeth) edgePos.push_back(ip);
        nearPos.clear();
    }
    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "初步生长提取用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();
    //再次生长分割，提取出包
    pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setInputCloud(cloud);
    reg.setIndices (inliers);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(10.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "二次生长分割滤波用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();
    std::vector<int>& bagindices = clusters[0].indices;
    cv::Mat image(rows, cols, CV_8UC1, cv::Scalar(0));
    cv::Mat image_process(rows, cols, CV_8UC1);

    for(auto i : bagindices){
        cloud->points[i].r=255;cloud->points[i].g=255;cloud->points[i].b=0;
        cloud_filtered->points[i]=cloud->points[i];
        image.data[i]=255;
    }

    //对图像处理出边缘
    cv::Mat ker = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(image, image_process, ker, cv::Point(-1,-1), 2);

    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "图像膨胀处理用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();

    cv::Canny(image_process, image_process,30,80);

    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "图像Canny处理用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();

    std::list<int> seedPos;
    for(int i=0; i<rows*cols; i++){
        if( image_process.data[i]==255){
            seedPos.push_back(i);
        }
    }
    //对边缘图聚类
    unsigned char* imdata=image_process.data;
    std::vector<std::vector<int>> posClusters;
    while(seedPos.empty() == false){
        int ip = seedPos.back();
        seedPos.pop_back();
        std::vector<int> cluster_res;
        cluster_res.push_back(ip);
        std::vector<int> cur_cluster;
        cur_cluster.push_back(ip);
        while(cur_cluster.empty() == false) {
            ip = cur_cluster.back();
            cur_cluster.pop_back();
            std::vector<int> nearPos;
            nearPos.push_back(ip - cols - 1);
            nearPos.push_back(ip - cols);
            nearPos.push_back(ip - cols + 1);
            nearPos.push_back(ip - 1);
            nearPos.push_back(ip + 1);
            nearPos.push_back(ip + cols - 1);
            nearPos.push_back(ip + cols);
            nearPos.push_back(ip + cols + 1);

            for (auto np : nearPos) {
                if (imdata[np] == 255){
                    std::list<int>::iterator itnp = std::find(seedPos.begin(),seedPos.end(), np);
                    if(itnp!= seedPos.end()) {
                        cur_cluster.push_back(np);
                        cluster_res.push_back(np);
                        seedPos.erase(itnp);
                    }
                }
            }
            nearPos.clear();
        }
        posClusters.push_back(cluster_res);
        cluster_res.clear();
    }

    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "边缘聚类用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();

    if( posClusters.size() < 2){
        //图像边缘聚类小于2，没有洞。
        printf("edge cluster less than 2, no hole on the image!!!!!!!!\n");
        return;
    }

    //从大到小冒泡排序
    std::vector<std::vector<int>*> sortClusters;
    for(std::vector<int>& i : posClusters){
        sortClusters.push_back(&i);
    }
    for(int i=sortClusters.size()-1; i>0; i--){
        for(int j=0;j<i; j++){
            if(sortClusters[j]->size() < sortClusters[j+1]->size()){
                std::vector<int>*tmp = sortClusters[j];
                sortClusters[j]=sortClusters[j+1];
                sortClusters[j+1]=tmp;
            }
        }
    }
    printf("totle cluster %lu\n", posClusters.size());
    int color[10]={55,127,255,255,255, 255};
    for(int i=0; i< sortClusters.size(); i++){
        std::vector<int>& clust = *sortClusters[i];
        for(auto p : clust){
            imdata[p]=color[i];
        }
        printf("size%d, %lu\n", i, clust.size());
    }
    //获得洞在图像的位置x,y
    int x=0, y=0;
    for(int pos: *sortClusters[1]){
        y+=(pos+1)/cols;x+=pos%cols;
    }

    x/=sortClusters[1]->size();
    y/=sortClusters[1]->size();
    printf("the pole center(%d,%d)\n", x,y);

    pcl::PointXYZ hole_pos(0,0,0);
    pcl::Normal hole_normal(0,0,0);
    int cnt=0;

    //向下寻找临近点
    for(int i=y*cols+x+cols; i<rows*cols;i+=cols){
        if(image.data[i] == 255){
            cnt++;
            hole_pos.x+=cloud->points[i].x;
            hole_pos.y+=cloud->points[i].y;
            hole_pos.z+=cloud->points[i].z;
            hole_normal.normal_x+=normals->points[i].normal_x;
            hole_normal.normal_y+=normals->points[i].normal_y;
            hole_normal.normal_z+=normals->points[i].normal_z;
            break;
        }
        image.data[i]=255;
    }
    //向上寻找临近点
    for(int i=y*cols+x-cols; i>=0;i-=cols){
        if(image.data[i] == 255){
            cnt++;
            hole_pos.x+=cloud->points[i].x;
            hole_pos.y+=cloud->points[i].y;
            hole_pos.z+=cloud->points[i].z;
            hole_normal.normal_x+=normals->points[i].normal_x;
            hole_normal.normal_y+=normals->points[i].normal_y;
            hole_normal.normal_z+=normals->points[i].normal_z;
            break;
        }
        image.data[i]=255;
    }
    //向左
    for(int i=y*cols+x-1; i>=y*cols;i--){
        if(image.data[i] == 255){
            cnt++;
            hole_pos.x+=cloud->points[i].x;
            hole_pos.y+=cloud->points[i].y;
            hole_pos.z+=cloud->points[i].z;
            hole_normal.normal_x+=normals->points[i].normal_x;
            hole_normal.normal_y+=normals->points[i].normal_y;
            hole_normal.normal_z+=normals->points[i].normal_z;
            break;
        }
        image.data[i]=255;
    }
    //向右
    for(int i=y*cols+x+1; i<(y+1)*cols;i++){
        if(image.data[i] == 255){
            cnt++;
            hole_pos.x+=cloud->points[i].x;
            hole_pos.y+=cloud->points[i].y;
            hole_pos.z+=cloud->points[i].z;
            hole_normal.normal_x+=normals->points[i].normal_x;
            hole_normal.normal_y+=normals->points[i].normal_y;
            hole_normal.normal_z+=normals->points[i].normal_z;
            break;
        }
        image.data[i]=255;
    }
    if(cnt < 4){
        cout << "洞不完整\n";
        return;
    }
    hole_pos.x/=cnt;
    hole_pos.y/=cnt;
    hole_pos.z/=cnt;
    hole_normal.normal_x/=cnt;
    hole_normal.normal_y/=cnt;
    hole_normal.normal_z/=cnt;

    end   = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "获取洞法线用时 "<< duration.count() <<" ms" << endl;
    start   = std::chrono::system_clock::now();

    cout << "延伸点: "<<cnt<<endl;
    cout<<"hole pos " << hole_pos << endl;
    cout<<"hole normal " << hole_normal << endl;
    double cos2cam = cosvec(hole_normal, pcl::Normal(0,0,-1));
    cout<< "洞方向和摄像头夹角余弦值 "<<cos2cam<<endl;
//    cv::imwrite("/home/isness/pcd files/test.jpg", image);
//    cv::imwrite("/home/isness/pcd files/test1.jpg", image_process);
    pcl::PointXYZ p1(hole_pos.x+hole_normal.normal_x, hole_pos.y+hole_normal.normal_y,hole_pos.z+hole_normal.normal_z);
    v1->addArrow(hole_pos, p1, 1,1,0);
    v2->addArrow(hole_pos, p1, 1,1,0);

//    int cnt=0;
//    double tx=0,ty=0,tz=0,tnx=0,tny=0,tnz=0;
//    for(int i=0; i<rows*cols; i++){
//        if(indices[i] == 1 ){
//            pcl::PointXYZRGBA& p = cloud->points[i];
//            pcl::Normal&n=normals->points[i];
//            cnt++;
////            printf("x%.2f y%.2f z%.2f nx%.2f ny%.2f nz%.2f\n", p.x, p.y, p.z, n.normal_x, n.normal_y, n.normal_z);
//            tx+=p.x;ty+=p.y;tz+=p.z;tnx+=n.normal_x;tny+=n.normal_y;tnz+=n.normal_z;
//            p.r=255;p.g=255;p.b=255;
//        }
//    }
//    tx/=cnt;ty/=cnt;tz/=cnt;tnx/=cnt;tny/=cnt;tnz/=cnt;
//    double a=tnx,b=tny,c=tnz,d=-a*tx-b*ty-c*tz;
//    for(int i=0; i<rows*cols; i++){
//        if(indices[i] == 1){
////            cloud_filtered->points[i].x-=tx;
////            cloud_filtered->points[i].y-=ty;
////            cloud_filtered->points[i].z-=tz;
//        }
//    }
//    for(auto pos : edgePos){
//        cloud_filtered->points[pos] = cloud->points[pos];
//    }
//    printf("tx%.2f ty%.2f tz%.2f tnx%.2f tny%.2f tnz%.2f\n", tx, ty, tz, tnx, tny, tnz);
//    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//    transform_2.translation() << -tx, -ty, -tz;
//    transform_2.rotate (Eigen::AngleAxisf (-atan2(tnx, tnz), Eigen::Vector3f::UnitY()));
//    transform_2.rotate (Eigen::AngleAxisf (M_PI_4, Eigen::Vector3f::UnitY()));
//    pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_2);
//    printf("the num of bag %d edge %lu\n", cnt, edgePos.size());

}

void visual_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clouds) {
    std::vector<pcl::visualization::CloudViewer *> viewers;
    int cnt = 0;
    for (auto it : clouds) {
        pcl::visualization::CloudViewer *viewer = new pcl::visualization::CloudViewer(std::to_string(cnt++));
        viewer->showCloud(it);
        viewers.push_back(viewer);
    }
    for (;;) {
        for (int i = 0; i < viewers.size(); i++) {
            if (viewers[i]->wasStopped()) return;
        }
    }

}

void cloud_save_rgb(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    cv::Mat rgb(cloud->height, cloud->width, CV_8UC3);
    for (int i = 0; i < cloud->height; i++) {
        for (int j = 0; j < cloud->width; j++) {
            rgb.at<cv::Vec3b>(i, j, 0) = cloud->points[i * cloud->width + j].r;
            rgb.at<cv::Vec3b>(i, j, 1) = cloud->points[i * cloud->width + j].g;
            rgb.at<cv::Vec3b>(i, j, 2) = cloud->points[i * cloud->width + j].b;

        }

    }
//    cv::imwrite("/home/isness/pcd files/test.jpg", rgb);
//    cv::imshow("test",rgb);
//    cv::waitKey(0);
}

int main(int argc, char **argv) {
//    ros::init(argc, argv, "myviewer",ros::init_options::AnonymousName);
//    if(!ros::ok()){ return 0; }
//    std::string topicColor="/kinect2/sd/image_color_rect";
//    std::string topicDepth="/kinect2/sd/image_depth";
//    Receiver receiver(topicColor, topicDepth, true, false);
//    OUT_INFO("start node"" myviewer");
//    receiver.run(Receiver::CLOUD);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>()), cloud1(
            new pcl::PointCloud<pcl::PointXYZRGBA>());


    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("/home/oym/pcd files/scene-1.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    cloud1->height = cloud->height;
    cloud1->width = cloud->width;
    cloud1->is_dense = false;
    cloud1->points.resize(cloud->height * cloud->width);
    process_input(argc, argv);

//    cloud_save_rgb(cloud);

//    pcl_filter_seg_cloud(cloud, cloud1);
    pcl::visualization::PCLVisualizer::Ptr visualizer(
            new pcl::visualization::PCLVisualizer("cloud")), visualizer1(
            new pcl::visualization::PCLVisualizer("cloud_filtered"));


    visualizer->addPointCloud(cloud, "cloudName");
//    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloudName");
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(0, 0);
    visualizer->setSize(cloud->width, cloud->height);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
//    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    visualizer1->addPointCloud(cloud1, "cloudName1");
    visualizer1->initCameraParameters();
    visualizer1->setBackgroundColor(0, 0, 0);
    visualizer1->setPosition(0, 0);
    visualizer1->setSize(cloud->width, cloud->height);
    visualizer1->setCameraPosition(0, 0, 0, 0, -1, 0);

    pcl_seg_obj(cloud, cloud1, visualizer, visualizer1);
    visualizer->updatePointCloud(cloud, "cloudName");
    visualizer1->updatePointCloud(cloud1, "cloudName1");
    while (true) {

//        visualizer->updatePointCloud()
        visualizer1->spinOnce(10);
        visualizer->spinOnce(10);
    }

    return 0;
}

