#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include<vector>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;//�����У�������������pointcloud::Ptr��pointcloud


void duiying_point(pointcloud::Ptr input_cloud, pointcloud::Ptr target_cloud)//��Ӧ������
{
	//��ʼ������
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(input_cloud);
	core.setInputTarget(target_cloud);
	pcl::Correspondences all_correspondences;//ȫ����Ӧ��
	//core.determineCorrespondences(all_correspondences,6);//ȷ�����������Ŀ�����֮��Ķ�Ӧ��ϵ��

	core.determineReciprocalCorrespondences(all_correspondences);   //ȷ�����������Ŀ�����֮��Ľ�����Ӧ��ϵ��
	float sum = 0.0, rmse;
	vector<float>Co;
	for (size_t j = 0; j < all_correspondences.size(); j++) 
	{
		sum += all_correspondences[j].distance;
		Co.push_back(all_correspondences[j].distance);//push_back��������vector����β�����all_correspondences[j].distance����GoΪ��Ӧ�������	
		//vector<int> vec;
		//vec.push_back(10);��vector����β�����10
	}
	rmse = sqrt(sum / all_correspondences.size());
	vector<float>::iterator max = max_element(Co.begin(), Co.end());//������(iteratorΪ����vector�ĵ�����)���������Ԫ�ز�������
	vector<float>::iterator min = min_element(Co.begin(), Co.end());//max_element��ѯ���ֵ����λ�ã�min_element��ѯ��Сֵ
	cout << "ƥ���Ը���" << all_correspondences.size() << endl;
	cout << "��Ӧ��������ֵ" << sqrt(*max) * 100 << "����" << endl;
	cout << "��Ӧ�������Сֵ" << sqrt(*min) * 100 << "����" << endl;
	cout << "��Ӧ����������" << rmse * 100 << "����" << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//��viewer����Ϊ���ܹ���ָ��
	//-----------�����������ɫ-------------------------
	viewer0->setBackgroundColor(0.3, 0.3, 0.3);
	viewer0->addCoordinateSystem(0.1);//���������
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(input_cloud, 255, 0, 0);//Դ������ɫ
	viewer0->addPointCloud<pcl::PointXYZ>(input_cloud, source_color, "source cloud");//��������ӵ��Ӵ�
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//���õ������Ӵ��е���ʾ��ʽ����Ⱦ���ԣ���С

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 0, 255, 0);//Ŀ����ƺ�ɫ
	viewer0->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	
	//��Ӧ��ϵ���ӻ�
	viewer0->addCorrespondences<pcl::PointXYZ>(input_cloud, target_cloud, all_correspondences, "correspondence");
	//viewer->initCameraParameters();
	while (!viewer0->wasStopped())
	{
		viewer0->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	
}







