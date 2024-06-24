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
#include "seg_obj.h"

#include <cv_bridge/cv_bridge.h>
//#include <kinect2_bridge/kinect2_definitions.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


static void pcl_filter_seg_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);

static void visual_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds);

void process_input(int argc, char **pString);

int num = 0;

void process_input(int argc, char **pString) {
    if (argc > 1)num = std::stoi(pString[1]);
    else num = 50;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

double DistanceOfPointToLine(pcl::PointXYZ *a, pcl::PointXYZ *b, pcl::PointXYZ *s) {
    double ab = sqrt(pow((a->x - b->x), 2.0) + pow((a->y - b->y), 2.0) + pow((a->z - b->z), 2.0));
    double as = sqrt(pow((a->x - s->x), 2.0) + pow((a->y - s->y), 2.0) + pow((a->z - s->z), 2.0));
    double bs = sqrt(pow((s->x - b->x), 2.0) + pow((s->y - b->y), 2.0) + pow((s->z - b->z), 2.0));
    double cos_A = (as * as + ab * ab - bs * bs) / (2 * ab * as);
    double sin_A = sqrt(1 - cos_A * cos_A);
    return as * sin_A;
}

double distance(pcl::PointXYZRGB &a, pcl::PointXYZRGB &b) {
    double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return sqrt(x * x + y * y + z * z);
}

double cosvec(const pcl::Normal &nor1, const pcl::Normal &nor2) {
    float x1 = nor1.normal_x,
            x2 = nor2.normal_x,
            y1 = nor1.normal_y,
            y2 = nor2.normal_y,
            z1 = nor1.normal_z,
            z2 = nor2.normal_z;
    return (x1 * x2 + y1 * y2 + z1 * z2) / sqrt(x1 * x1 + y1 * y1 + z1 * z1) / sqrt(x2 * x2 + y2 * y2 + z2 * z2);
}

extern pcl::visualization::CloudViewer cloud_viewer;

cv::Mat image(480, 640, CV_8UC1, cv::Scalar(0));
cv::Mat image_process(480, 640, CV_8UC1);
void pcl_seg_obj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered,
                 pcl::PointXY &p1_2d,
                 pcl::PointXY &p2_2d,
                 pcl::PointXYZ &p1_3d,
                 pcl::PointXYZ &p2_3d,
                 pcl::PointXYZ &hole_point,
                 pcl::PointXYZ &hole_vec) {

    int rows = cloud->height, cols = cloud->width;
    //pcl::PointXY p1_2d=pcl::PointXY(222, 136);
    //pcl::PointXY p2_2d=pcl::PointXY(241, 156);
//    pcl::PointXY p1_2d;
//    p1_2d.x = 222;
//    p1_2d.y = 136;
//
//    pcl::PointXY p2_2d;
//    p2_2d.x = 241;
//    p2_2d.y = 156;

//    pcl::PointXYZ p1_3d(cloud->points[cols * p1_2d.y + p1_2d.x].x, cloud->points[cols * p1_2d.y + p1_2d.x].y,
//                        cloud->points[cols * p1_2d.y + p1_2d.x].z);
//    pcl::PointXYZ p2_3d(cloud->points[cols * p2_2d.y + p2_2d.x].x, cloud->points[cols * p2_2d.y + p2_2d.x].y,
//                        cloud->points[cols * p2_2d.y + p2_2d.x].z);

    pcl::PointXYZ pv_3d(p2_3d.x - p1_3d.x, p2_3d.y - p1_3d.y, p2_3d.z - p1_3d.z);
//    printf("p1(%.2f,%.2f,%.2f),p2(%.2f,%.2f,%.2f)\n",p1_3d.x,p1_3d.y,p1_3d.z,p2_3d.x,p2_3d.y,p2_3d.z);
//    float pv_dis = sqrt(pv_3d.x * pv_3d.x + pv_3d.y * pv_3d.y + pv_3d.z * pv_3d.z);
//    pv_3d.x /= pv_dis;
//    pv_3d.y /= pv_dis;
//    pv_3d.z /= pv_dis;
    int scale_pix = 10;
    float radius = 0.2;
    float vec_pix_x = (p2_2d.x - p1_2d.x) / sqrt(pow(p2_2d.x - p1_2d.x, 2) + pow(p2_2d.y - p1_2d.y, 2));
    float vec_pix_y = (p2_2d.y - p1_2d.y) / sqrt(pow(p2_2d.x - p1_2d.x, 2) + pow(p2_2d.y - p1_2d.y, 2));
    int scan_cnt = 0;
    int scan_thresh = 400;

    auto start = std::chrono::system_clock::now();

    //索引出对应的包， 0为未判断，1是包，-1不是包
    int indices[rows * cols];
    //初始化数组为0，注意sizeof(int)
    memset(indices, 0, rows * cols * sizeof(int));
    std::list<int> indexPos;

    //索引去除手臂范围外的点
//    pcl::PointIndices::Ptr rele_inliers(new pcl::PointIndices());
    int i0,i1,j0,j1;
    if(vec_pix_x<0){
        j0=p1_2d.x;j1=cols;
    }else{
        j0=0;j1=p1_2d.x;
    }
    if(vec_pix_y<0){
        i0=p1_2d.y;i1=rows;
    }else{
        i0=0;i1=p1_2d.y;
    }
    const float not_num = std::numeric_limits<float>::quiet_NaN();
    for (int i = i0; i < i1; i++) {
        for(int j=0;j<cols; j++){
            int pos = i*cols+j;
            cloud->points[pos].x=cloud->points[pos].y=cloud->points[pos].z=not_num;
        }
    }
    if(i0==0){
        i0=i1;i1=rows;
    }else{
        i0=0;i1=p1_2d.y;
    }
    for (int i = i0; i < i1; i++) {
        for(int j=j0;j<j1; j++){
            int pos = i*cols+j;
            cloud->points[pos].x=cloud->points[pos].y=cloud->points[pos].z=not_num;
        }
    }

    //初步索引部分目标点
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    int dir = vec_pix_y > 0 ? 1 : -1;
    for (float x = p2_2d.x + 2 * scale_pix * vec_pix_x / std::abs(vec_pix_y), y = p2_2d.y + 2 * scale_pix;
         x < cols - scale_pix && y < rows && x > scale_pix && y > 0; x += vec_pix_x / std::abs(vec_pix_y), y += dir) {
        for (int j = -scale_pix, pos = (int) (int(y) * cols + int(x) - scale_pix); j < scale_pix; j++, pos++) {

            if (std::isnan(cloud->points[pos].x)) {
                indices[pos] = -1;
                continue;
            }

            pcl::PointXYZ tmp(cloud->points[pos].x, cloud->points[pos].y, cloud->points[pos].z);
            double dis = DistanceOfPointToLine(&p1_3d, &p2_3d, &tmp);

            if (dis < radius) {
                cloud->points[pos].r = 255;
                cloud->points[pos].g = 0;
                cloud->points[pos].b = 0;
//                printf("p%d w%d h%d x%.2f y%.1f j%d dis%.2f cnt%d\n", pos, cols, rows, x, y,  j, dis, scan_cnt);
                scan_cnt++;
                indices[pos] = 1;
                inliers->indices.push_back(pos);
                indexPos.push_back(pos);
            } else {

            }
//            cloud_viewer.showCloud(cloud);
//            cloud_viewer.wasStopped();
        }
        if (scan_cnt > scan_thresh) break;
    }

    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "初步索引部分目标点用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();
    //法线估计
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
//    normal_estimator.setIndices(rele_inliers);
    normal_estimator.setKSearch(140);
    cout << "开始计算法线\n";
    normal_estimator.compute(*normals);
    cout << "计算法线结束\n";
    int maxpos = rows * cols;
    double costh = cos(8.0 * M_PI/ 180 );
    double disth = 0.015;
//    int edgeth = 11;
    std::vector<int> edgePos;

    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "计算法线用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();
    //初步生长
    while (!indexPos.empty()) {
        int ip = indexPos.front();
        indexPos.pop_front();
        if (ip > maxpos - cols - 2) continue;

        std::vector<int> nearPos;

//        nearPos.push_back(ip - 2*cols - 2);
//        nearPos.push_back(ip - 2*cols - 1);
//        nearPos.push_back(ip - 2*cols );
//        nearPos.push_back(ip - 2*cols +1);
//        nearPos.push_back(ip - 2*cols +2);
//        nearPos.push_back(ip - cols - 2);
        nearPos.push_back(ip - cols - 1);
        nearPos.push_back(ip - cols);
        nearPos.push_back(ip - cols + 1);
//        nearPos.push_back(ip - cols + 2);
//        nearPos.push_back(ip - 3);
//        nearPos.push_back(ip - 2);
        nearPos.push_back(ip - 1);
        nearPos.push_back(ip + 1);
//        nearPos.push_back(ip + 2);
//        nearPos.push_back(ip + 3);
        nearPos.push_back(ip + cols - 1);
//        nearPos.push_back(ip + cols - 2);
        nearPos.push_back(ip + cols);
        nearPos.push_back(ip + cols + 1);
//        nearPos.push_back(ip + cols + 2);
//        nearPos.push_back(ip + 2*cols - 2);
//        nearPos.push_back(ip + 2*cols - 1);
//        nearPos.push_back(ip + 2*cols );
//        nearPos.push_back(ip + 2*cols +1);
//        nearPos.push_back(ip + 2*cols +2);

        pcl::PointXYZRGB &seed = cloud->points[ip];
        pcl::Normal &nor = normals->points[ip];
//        int nearNum = 0;
        for (int i = 0; i < nearPos.size(); i++) {
            int np = nearPos[i];
            pcl::PointXYZRGB &seed1 = cloud->points[np];
            pcl::Normal &nor1 = normals->points[np];
            if (std::isnan(seed1.x)) {
                indices[np] = -1;
                continue;
            }
            if (indices[np] == 0) {
                double dis = distance(seed1, seed);
                double cv = cosvec(nor, nor1);
                if (dis < disth && abs(cv) > costh) {
//                    nearNum++;
//                    printf("ip%d np%d dis%.2f cosvec%.2f costh%.2f\n", ip, np, dis, cv, costh);
                    indices[np] = 1;
                    cloud->points[np].r = cloud->points[np].g = 255;cloud->points[np].b=0;
                    inliers->indices.push_back(np);
                    indexPos.push_back(np);
                } else {
                    indices[np] = -1;
                }
            }
//            else if(indices[np]==1) nearNum++;
        }
//        if(nearNum < edgeth) edgePos.push_back(ip);
        nearPos.clear();
    }
    cloud_viewer.showCloud(cloud);
    cloud_viewer.wasStopped();
    cout<<"初步生长得到点的数量 "<<inliers->indices.size()<<endl;
    if(inliers->indices.size()<1) return;
//    cloud_viewer.showCloud(cloud);
//    cloud_viewer.wasStopped();

    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "初步生长提取用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();
    //再次生长分割，提取出包
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    reg.setIndices(inliers);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(9.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "二次生长分割滤波用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();

    if (clusters.size() < 1) {
        cout << "二次生长分割失败\n";
        return;
    }
    std::vector<int> &bagindices = clusters[0].indices;

    memset(image.data, 0, rows*cols);
    memset(image_process.data , 0, rows*cols);
    for (auto i : bagindices) {
//        cloud->points[i].r = 255;
//        cloud->points[i].g = 255;
//        cloud->points[i].b = 0;
        cloud_filtered->points[i] = cloud->points[i];
        image.data[i] = 255;
    }

    cout << "开始处理图像边缘\n";
    //对图像处理出边缘
    cv::imshow("raw", image);
    cv::waitKey(1);
    cv::Mat ker = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(image, image_process, ker, cv::Point(-1, -1), 1);
    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "图像膨胀处理用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();
    cv::imshow("dilate", image_process);
    cv::waitKey(1);
    cv::Canny(image_process, image_process, 30, 80);
    cv::imshow("canny", image_process);
    cv::waitKey(1);
    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "图像Canny处理用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();

    std::vector<int> seedPos;
    for (int i = 0; i < rows * cols; i++) {
        if (image_process.data[i] == 255) {
            seedPos.push_back(i);
        }
    }
    //对边缘图聚类
    unsigned char *imdata = image_process.data;
    std::vector<std::vector<int>> posClusters;
    while (seedPos.empty() == false) {
        int ip = seedPos.back();
        seedPos.pop_back();
        std::vector<int> cluster_res;
        cluster_res.push_back(ip);
        std::vector<int> cur_cluster;
        cur_cluster.push_back(ip);
        while (cur_cluster.empty() == false) {
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
                if (imdata[np] == 255) {
                    std::vector<int>::iterator itnp = std::find(seedPos.begin(), seedPos.end(), np);
                    if (itnp != seedPos.end()) {
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

    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "边缘聚类用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();

    if (posClusters.size() < 2) {
        //图像边缘聚类小于2，没有洞。
        printf("图像边缘聚类小于2，没有洞!!!!!!!!\n");
        return;
    }

    //从大到小冒泡排序
    std::vector<std::vector<int> *> sortClusters;
    for (std::vector<int> &i : posClusters) {
        sortClusters.push_back(&i);
    }
    for (int i = sortClusters.size() - 1; i > 0; i--) {
        for (int j = 0; j < i; j++) {
            if (sortClusters[j]->size() < sortClusters[j + 1]->size()) {
                std::vector<int> *tmp = sortClusters[j];
                sortClusters[j] = sortClusters[j + 1];
                sortClusters[j + 1] = tmp;
            }
        }
    }
    printf("totle cluster %lu\n", posClusters.size());
//    int color[10] = {55, 127, 255, 255, 255, 255};
//    for (int i = 0; i < sortClusters.size(); i++) {
//        std::vector<int> &clust = *sortClusters[i];
//        for (auto p : clust) {
//            imdata[p] = color[i];
//        }
//        printf("size%d, %lu\n", i, clust.size());
//    }
    //获得洞在图像的位置x,y
    int x = 0, y = 0;
    for (int pos: *sortClusters[1]) {
        y += (pos + 1) / cols;
        x += pos % cols;
    }

    x /= sortClusters[1]->size();
    y /= sortClusters[1]->size();
    printf("the pole center(%d,%d)\n", x, y);

    pcl::PointXYZ hole_pos(0, 0, 0);
    pcl::Normal hole_normal(0, 0, 0);
    int cnt = 0;

    //向下寻找临近点
    for (int i = y * cols + x + cols; i < rows * cols; i += cols) {
        if (image.data[i] == 255) {
            cnt++;
            hole_pos.x += cloud->points[i].x;
            hole_pos.y += cloud->points[i].y;
            hole_pos.z += cloud->points[i].z;
            hole_normal.normal_x += normals->points[i].normal_x;
            hole_normal.normal_y += normals->points[i].normal_y;
            hole_normal.normal_z += normals->points[i].normal_z;
            break;
        }
        image.data[i] = 255;
    }
    //向上寻找临近点
    for (int i = y * cols + x - cols; i >= 0; i -= cols) {
        if (image.data[i] == 255) {
            cnt++;
            hole_pos.x += cloud->points[i].x;
            hole_pos.y += cloud->points[i].y;
            hole_pos.z += cloud->points[i].z;
            hole_normal.normal_x += normals->points[i].normal_x;
            hole_normal.normal_y += normals->points[i].normal_y;
            hole_normal.normal_z += normals->points[i].normal_z;
            break;
        }
        image.data[i] = 255;
    }
    //向左
    for (int i = y * cols + x - 1; i >= y * cols; i--) {
        if (image.data[i] == 255) {
            cnt++;
            hole_pos.x += cloud->points[i].x;
            hole_pos.y += cloud->points[i].y;
            hole_pos.z += cloud->points[i].z;
            hole_normal.normal_x += normals->points[i].normal_x;
            hole_normal.normal_y += normals->points[i].normal_y;
            hole_normal.normal_z += normals->points[i].normal_z;
            break;
        }
        image.data[i] = 255;
    }
    //向右
    for (int i = y * cols + x + 1; i < (y + 1) * cols; i++) {
        if (image.data[i] == 255) {
            cnt++;
            hole_pos.x += cloud->points[i].x;
            hole_pos.y += cloud->points[i].y;
            hole_pos.z += cloud->points[i].z;
            hole_normal.normal_x += normals->points[i].normal_x;
            hole_normal.normal_y += normals->points[i].normal_y;
            hole_normal.normal_z += normals->points[i].normal_z;
            break;
        }
        image.data[i] = 255;
    }
    if (cnt < 4) {
        cout << "洞不完整\n";
        return;
    }
    hole_pos.x /= cnt;
    hole_pos.y /= cnt;
    hole_pos.z /= cnt;
    hole_normal.normal_x /= cnt;
    hole_normal.normal_y /= cnt;
    hole_normal.normal_z /= cnt;

    end = std::chrono::system_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << "获取洞法线用时 " << duration.count() << " ms" << endl;
    start = std::chrono::system_clock::now();

    cout << "延伸点: " << cnt << endl;
    cout << "hole pos " << hole_pos << endl;
    cout << "hole normal " << hole_normal << endl;
    hole_normal.normal_x/=10;hole_normal.normal_y/=10;hole_normal.normal_z/=10;
    double cos2cam = cosvec(hole_normal, pcl::Normal(0, 0, -1));
    cout << "洞方向和摄像头夹角余弦值 " << cos2cam << endl;
//    cv::imwrite("/home/isness/pcd files/test.jpg", image);
//    cv::imwrite("/home/isness/pcd files/test1.jpg", image_process);
    pcl::PointXYZ p1(hole_pos.x + hole_normal.normal_x, hole_pos.y + hole_normal.normal_y,
                     hole_pos.z + hole_normal.normal_z);
    hole_point = hole_pos;
    hole_vec = p1;
//    v1->addArrow(hole_pos, p1, 1,1,0);
//    v2->addArrow(hole_pos, p1, 1,1,0);

//    int cnt=0;
//    double tx=0,ty=0,tz=0,tnx=0,tny=0,tnz=0;
//    for(int i=0; i<rows*cols; i++){
//        if(indices[i] == 1 ){
//            pcl::PointXYZRGB& p = cloud->points[i];
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

void visual_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds) {
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

void cloud_save_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

//int main(int argc, char **argv) {
////    ros::init(argc, argv, "myviewer",ros::init_options::AnonymousName);
////    if(!ros::ok()){ return 0; }
////    std::string topicColor="/kinect2/sd/image_color_rect";
////    std::string topicDepth="/kinect2/sd/image_depth";
////    Receiver receiver(topicColor, topicDepth, true, false);
////    OUT_INFO("start node"" myviewer");
////    receiver.run(Receiver::CLOUD);
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>()), cloud1(
//            new pcl::PointCloud<pcl::PointXYZRGB>());
//
//
//    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/oym/pcd files/scene-1.pcd", *cloud) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//        return (-1);
//    }
//
//    cloud1->height = cloud->height;
//    cloud1->width = cloud->width;
//    cloud1->is_dense = false;
//    cloud1->points.resize(cloud->height * cloud->width);
//    process_input(argc, argv);
//
////    cloud_save_rgb(cloud);
//
////    pcl_filter_seg_cloud(cloud, cloud1);
//    pcl::visualization::PCLVisualizer::Ptr visualizer(
//            new pcl::visualization::PCLVisualizer("cloud")), visualizer1(
//            new pcl::visualization::PCLVisualizer("cloud_filtered"));
//
//
//    visualizer->addPointCloud(cloud, "cloudName");
////    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloudName");
//    visualizer->initCameraParameters();
//    visualizer->setBackgroundColor(0, 0, 0);
//    visualizer->setPosition(0, 0);
//    visualizer->setSize(cloud->width, cloud->height);
//    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
////    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);
//
//    visualizer1->addPointCloud(cloud1, "cloudName1");
//    visualizer1->initCameraParameters();
//    visualizer1->setBackgroundColor(0, 0, 0);
//    visualizer1->setPosition(0, 0);
//    visualizer1->setSize(cloud->width, cloud->height);
//    visualizer1->setCameraPosition(0, 0, 0, 0, -1, 0);
//
//    pcl_seg_obj(cloud, cloud1, visualizer, visualizer1);
//    visualizer->updatePointCloud(cloud, "cloudName");
//    visualizer1->updatePointCloud(cloud1, "cloudName1");
//    while (true) {
//
////        visualizer->updatePointCloud()
//        visualizer1->spinOnce(10);
//        visualizer->spinOnce(10);
//    }
//
//    return 0;
//}

