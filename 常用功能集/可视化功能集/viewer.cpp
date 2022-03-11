#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include"test.h"
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;



void visualize_cloud_windows(pointcloud::Ptr &source, pointcloud::Ptr &target)
{
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("bun045.pcd", *source);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("GD1251210(1.0)-3.pcd", *target);*/
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//将viewer定义为智能共享指针
	//-----------给点云添加颜色-------------------------
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source, 0, 255, 0);//设置点云颜色，法一
	viewer0->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud");//将点云添加到视窗
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//设置点云再视窗中的显示方式，渲染属性，大小

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
	viewer0->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	viewer0->addCoordinateSystem(0.1);//添加坐标轴




	//----------下一项----
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("display"));//窗一
	//设置背景颜色
	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	//添加点云
	viewer->addPointCloud(source, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");//设置点云颜色，法二
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("display"));//窗二
	//设置背景颜色
	viewer1->setBackgroundColor(0, 1, 1);
	//添加点云
	viewer1->addPointCloud(target, "cloud1");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
