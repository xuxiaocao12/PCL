#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include"test.h"
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

void windows_2cloud(pointcloud::Ptr &source, pointcloud::Ptr &target)	//一窗口显示两点云
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//将viewer定义为智能共享指针
	//-----------给点云添加颜色-------------------------
	viewer0->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source, 255, 0, 0);//源点云绿色
	viewer0->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud");//将点云添加到视窗
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//设置点云再视窗中的显示方式，渲染属性，大小

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 0, 255, 0);//目标点云红色
	viewer0->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	viewer0->addCoordinateSystem(0.1);//添加坐标轴
	
}
void visualize_one_cloud(pointcloud::Ptr &source)	//可视化单个点云----------按z字段进行渲染
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show_one_cloud"));//将viewer定义为智能共享指针
	viewer0->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(source, "z"); // 按照z字段进行渲染
	viewer0->addPointCloud<pcl::PointXYZ>(source, fildColor, "source cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//设置点云再视窗中的显示方式，渲染属性，大小
	viewer0->addCoordinateSystem(0.1);//添加坐标轴
}
void visualization01(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) //可视化带法线的点云---------------------
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "x");//按照z值进行渲染点的颜色

	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "bunny cloud");	// 添加需要显示的点云数据
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, "bunny cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "bunny cloud");// 设置点显示大小

	// 添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，1表示需要显示法向的点云间隔，即每1个点显示一次法向，0.003表示法向长度。
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.003, "normals");

	// Concatenate the XYZ and normal fields* 将点云数据与法向信息拼接
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //带法向量的点云。 pcl::PointNormal, pcl::PointXYZINormal, pcl::PointXYZRGBNormal
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); //点云拼接(Concatenate)。
	//Concatenate一种是点云的字段与维度完全一致，数量可以不同，两部分点云数量相加。另一种是点云的字段与维度不一致，但数量相同，进行维度的累加。
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud_with_normals, 1, 0.6, "normals");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


void visualize_cloud_windows(pointcloud::Ptr &source, pointcloud::Ptr &target)	//----------两窗口显示两点云---
{
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("bun045.pcd", *source);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("GD1251210(1.0)-3.pcd", *target);*/
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
