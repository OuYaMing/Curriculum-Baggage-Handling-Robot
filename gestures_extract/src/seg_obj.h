//
// Created by oym on 2020/11/5.
//

#ifndef SRC_SEG_OBJ_H
#define SRC_SEG_OBJ_H
#include <pcl/point_cloud.h>
void pcl_seg_obj(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered,
                 pcl::PointXY& p1_2d,
                 pcl::PointXY& p2_2d,
                 pcl::PointXYZ& p1_3d,
                 pcl::PointXYZ& p2_3d,
                 pcl::PointXYZ& hole_point,
                 pcl::PointXYZ& hole_vec);

#endif //SRC_SEG_OBJ_H
