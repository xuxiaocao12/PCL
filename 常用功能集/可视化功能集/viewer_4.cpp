#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\visualization\pcl_visualizer.h>
#include"test.h"
#include <vtkAutoInit.h>


using namespace std;

void visualize_4_cloud(pointcloud::Ptr &source, pointcloud::Ptr &target)
{
	//----------------------------- Visualizer ���ӻ�����ƣ��ɿ����ӻ���Ĺ��ܻ�������� ------------------------------
	///��ѡ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Multiple_Viewports")); //�����Ӵ����󣬶�����������ơ�3D Viewer��

	//-----�ӿ�1-----
	int v1(0);
	viewer->createViewPort(0.0, 0.5, 0.5, 1, v1);//�����Ӵ�λ�ã��ܴ�С1��1�������������Ϊ�ӿ���X�������Сֵ��Y�������Сֵ��X��������ֵ��Y��������ֵ���ӿ�ID
	viewer->setBackgroundColor(0, 0, 0, v1);//������ɫrgbȡֵ��Χ�� [0, 1] ������ [0, 255]����Ҫ����255��
	viewer->addText("one cloud", 10, 10, "v1 text", v1);//������ֲ���Ϊ���ģ�����vtk��񱨴�������
	viewer->addPointCloud(source, "sample_cloud0001", v1);//����һ��Ψһ���ַ�����ΪID��,���ô��ַ�����֤��������Ա��Ҳ�ܱ�־���øõ���---
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample_cloud0001");//���õ�����ɫ��0��1ȡֵ�������ַ�����ΪID��Ҫ��ǰ��һ��
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud0001");//���õ��С���ַ���ID��sample_cloud0001Ҫ��ǰ��һ��
	
	//-----�ӿ�2-----
	int v2(0);
	viewer->createViewPort(0.5, 0.5, 1, 1, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("two cloud", 10, 10, "v2 text", v2);
	/*���ô��ڵı�����ɫ�󣬴���һ����ɫ�������*/
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 0, 255, 0);//���õ�����ɫ��0��255ȡֵ��������ΪPointXYZ������PointXYZRGB�����򱨴�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source, 0, 0, 255);
	/* addPointCloud<>() ��ɶ���ɫ����������Ĵ��ݣ�Ϊ������ɫ */
	viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "sample cloud0002", v2);
	viewer->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud0002", v2);
	/*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");�����п���ʾ�����Դ�rgb��ɫ��Ϣ
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(cloud, "y");���ݵ��Ƶ�ĳ���ֶν�����ɫ��
	�������ֶ���x��y,z,normal_x��X�����ϵķ��ߣ�normal_y ��Y�����ϵķ��ߣ�normal_z ��Z�����ϵķ��ߣ�,rgb ����ɫ��,curvature �����ʣ�*/
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud0002");//���õ��С
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source cloud0002");//���õ��С

	//-----�ӿ�3-----
	int v3(0);
	viewer->createViewPort(0, 0, 0.5, 0.5, v3);
	viewer->setBackgroundColor(0, 0.5, 0, v3);
	viewer->addText("third windows", 10, 10, "v3 text", v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(source, 0, 0, 255);//���õ�����ɫ��0��255ȡֵ��������ɫ��ɫ��������ɫ
	viewer->addPointCloud<pcl::PointXYZ>(source, color3, "cloud0003", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud0003");//���õ��С

	//-----�ӿ�4-----
	int v4(0);
	viewer->createViewPort(0.5, 0, 1, 0.5, v4);
	viewer->setBackgroundColor(0.0, 255, 255, v4);
	viewer->addText("fourth", 10, 10, "v4 text", v4);
	viewer->addPointCloud(target, "sample_cloud4", v4);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample_cloud4");//���õ�����ɫ��0��1ȡֵ����
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud4");//���õ��С
	
	while (!viewer->wasStopped())//��ѡ����ֹ�����괰����һ�¾���ʧ
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
