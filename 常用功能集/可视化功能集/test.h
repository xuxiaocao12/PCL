#ifndef _TEST_H
#define _TEST_H

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

void visualize_cloud_windows(pointcloud::Ptr &source, pointcloud::Ptr &target);
void visualize_4_cloud(pointcloud::Ptr &source, pointcloud::Ptr &target);//viewer4.cppÀïµÄ

#endif

