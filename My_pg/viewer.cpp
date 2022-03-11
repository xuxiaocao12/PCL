#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include"test.h"
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

void windows_2cloud(pointcloud::Ptr &source, pointcloud::Ptr &target)	//һ������ʾ������
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//��viewer����Ϊ���ܹ���ָ��
	//-----------�����������ɫ-------------------------
	viewer0->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source, 255, 0, 0);//Դ������ɫ
	viewer0->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud");//��������ӵ��Ӵ�
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//���õ������Ӵ��е���ʾ��ʽ����Ⱦ���ԣ���С

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 0, 255, 0);//Ŀ����ƺ�ɫ
	viewer0->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	viewer0->addCoordinateSystem(0.1);//���������
	
}
void visualize_one_cloud(pointcloud::Ptr &source)	//���ӻ���������----------��z�ֶν�����Ⱦ
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show_one_cloud"));//��viewer����Ϊ���ܹ���ָ��
	viewer0->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(source, "z"); // ����z�ֶν�����Ⱦ
	viewer0->addPointCloud<pcl::PointXYZ>(source, fildColor, "source cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//���õ������Ӵ��е���ʾ��ʽ����Ⱦ���ԣ���С
	viewer0->addCoordinateSystem(0.1);//���������
}
void visualization01(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) //���ӻ������ߵĵ���---------------------
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "x");//����zֵ������Ⱦ�����ɫ

	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "bunny cloud");	// �����Ҫ��ʾ�ĵ�������
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, "bunny cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "bunny cloud");// ���õ���ʾ��С

	// �����Ҫ��ʾ�ĵ��Ʒ���cloudΪԭʼ����ģ�ͣ�normalΪ������Ϣ��1��ʾ��Ҫ��ʾ����ĵ��Ƽ������ÿ1������ʾһ�η���0.003��ʾ���򳤶ȡ�
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.003, "normals");

	// Concatenate the XYZ and normal fields* �����������뷨����Ϣƴ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //���������ĵ��ơ� pcl::PointNormal, pcl::PointXYZINormal, pcl::PointXYZRGBNormal
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); //����ƴ��(Concatenate)��
	//Concatenateһ���ǵ��Ƶ��ֶ���ά����ȫһ�£��������Բ�ͬ�������ֵ���������ӡ���һ���ǵ��Ƶ��ֶ���ά�Ȳ�һ�£���������ͬ������ά�ȵ��ۼӡ�
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud_with_normals, 1, 0.6, "normals");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


void visualize_cloud_windows(pointcloud::Ptr &source, pointcloud::Ptr &target)	//----------��������ʾ������---
{
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("bun045.pcd", *source);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("GD1251210(1.0)-3.pcd", *target);*/
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
