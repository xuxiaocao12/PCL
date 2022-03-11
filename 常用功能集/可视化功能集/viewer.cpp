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
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//��viewer����Ϊ���ܹ���ָ��
	//-----------�����������ɫ-------------------------
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source, 0, 255, 0);//���õ�����ɫ����һ
	viewer0->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud");//��������ӵ��Ӵ�
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//���õ������Ӵ��е���ʾ��ʽ����Ⱦ���ԣ���С

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
	viewer0->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	viewer0->addCoordinateSystem(0.1);//���������




	//----------��һ��----
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("display"));//��һ
	//���ñ�����ɫ
	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	//��ӵ���
	viewer->addPointCloud(source, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");//���õ�����ɫ������
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("display"));//����
	//���ñ�����ɫ
	viewer1->setBackgroundColor(0, 1, 1);
	//��ӵ���
	viewer1->addPointCloud(target, "cloud1");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
