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
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;//必须有！！！！！定义pointcloud::Ptr的pointcloud


void duiying_point(pointcloud::Ptr input_cloud, pointcloud::Ptr target_cloud)//对应点连线
{
	//初始化对象
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(input_cloud);
	core.setInputTarget(target_cloud);
	pcl::Correspondences all_correspondences;//全部对应点
	//core.determineCorrespondences(all_correspondences,6);//确定输入点云与目标点云之间的对应关系：

	core.determineReciprocalCorrespondences(all_correspondences);   //确定输入点云与目标点云之间的交互对应关系。
	float sum = 0.0, rmse;
	vector<float>Co;
	for (size_t j = 0; j < all_correspondences.size(); j++) 
	{
		sum += all_correspondences[j].distance;
		Co.push_back(all_correspondences[j].distance);//push_back函数，在vector容器尾部添加all_correspondences[j].distance。即Go为对应点的容器	
		//vector<int> vec;
		//vec.push_back(10);在vector容器尾部添加10
	}
	rmse = sqrt(sum / all_correspondences.size());
	vector<float>::iterator max = max_element(Co.begin(), Co.end());//迭代器(iterator为容器vector的迭代器)检查容器内元素并遍历。
	vector<float>::iterator min = min_element(Co.begin(), Co.end());//max_element查询最大值所在位置，min_element查询最小值
	cout << "匹配点对个数" << all_correspondences.size() << endl;
	cout << "对应点距离最大值" << sqrt(*max) * 100 << "厘米" << endl;
	cout << "对应点距离最小值" << sqrt(*min) * 100 << "厘米" << endl;
	cout << "对应点均方根误差" << rmse * 100 << "厘米" << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//将viewer定义为智能共享指针
	//-----------给点云添加颜色-------------------------
	viewer0->setBackgroundColor(0.3, 0.3, 0.3);
	viewer0->addCoordinateSystem(0.1);//添加坐标轴
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(input_cloud, 255, 0, 0);//源点云绿色
	viewer0->addPointCloud<pcl::PointXYZ>(input_cloud, source_color, "source cloud");//将点云添加到视窗
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//设置点云再视窗中的显示方式，渲染属性，大小

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 0, 255, 0);//目标点云红色
	viewer0->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	
	//对应关系可视化
	viewer0->addCorrespondences<pcl::PointXYZ>(input_cloud, target_cloud, all_correspondences, "correspondence");
	//viewer->initCameraParameters();
	while (!viewer0->wasStopped())
	{
		viewer0->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	
}







