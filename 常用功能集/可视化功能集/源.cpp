#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include"test.h"
#include <vtkAutoInit.h>

using namespace std;

int
main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bun045.pcd", *source);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("GD1251210(1.0)-3.pcd", *target);
	//（ctrl+d可直接复制一行）
	//visualize_cloud_windows(source, target);
	  visualize_4_cloud(source, target);

	return 0;
}
