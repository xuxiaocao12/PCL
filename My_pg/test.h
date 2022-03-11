#ifndef _TEST_H
#define _TEST_H

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
void windows_2cloud(pointcloud::Ptr &source, pointcloud::Ptr &target);
void visualize_cloud_windows(pointcloud::Ptr &source, pointcloud::Ptr &target);//viewer.cpp里的
void visualize_4_cloud(pointcloud::Ptr &source, pointcloud::Ptr &target);//viewer4.cpp里的
void visualization01(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);//可视化带法向量的点云，viewer.cpp里
void visualize_one_cloud(pointcloud::Ptr &source);	//可视化单个点云，viewer.cpp里
void visualize_pcd(pointcloud::Ptr &pcd_src, pointcloud::Ptr &pcd_tgt, pointcloud::Ptr &pcd_final);//ransac
void my_ransac(pointcloud::Ptr source, pointcloud::Ptr t_cloud, pointcloud::Ptr align);
void duiying_point(pointcloud::Ptr source, pointcloud::Ptr t_cloud);
#endif

