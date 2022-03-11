#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\visualization\pcl_visualizer.h>
#include"test.h"
#include <vtkAutoInit.h>
//在主程序中调用即可visualize_4_cloud(source, target);

using namespace std;

void visualize_4_cloud(pointcloud::Ptr &source, pointcloud::Ptr &target)
{
	//----------------------------- Visualizer 可视化多点云（可看可视化类的功能汇总来查表） ------------------------------
	///必选
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Multiple_Viewports")); //创建视窗对象，定义标题栏名称“3D Viewer”

	//-----视口1-----
	int v1(0);
	viewer->createViewPort(0.0, 0.5, 0.5, 1, v1);//设置视窗位置，总大小1乘1。五个参数依次为视口在X坐标的最小值、Y坐标的最小值、X坐标的最大值、Y坐标的最大值、视口ID
	viewer->setBackgroundColor(0, 0, 0, v1);//背景颜色rgb取值范围是 [0, 1] 而不是 [0, 255]。需要除以255：
	viewer->addText("one cloud", 10, 10, "v1 text", v1);//添加文字不可为中文，否则vtk疯狂报错！！！！
	viewer->addPointCloud(source, "sample_cloud0001", v1);//定义一个唯一的字符串作为ID号,利用此字符串保证在其他成员中也能标志引用该点云---
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample_cloud0001");//设置点云颜色（0到1取值法）。字符串作为ID号要与前面一致
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud0001");//设置点大小。字符串ID号sample_cloud0001要与前面一致
	
	//-----视口2-----
	int v2(0);
	viewer->createViewPort(0.5, 0.5, 1, 1, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("two cloud", 10, 10, "v2 text", v2);
	/*设置窗口的背景颜色后，创建一个颜色处理对象*/
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 0, 255, 0);//设置点云颜色（0到255取值法）。需为PointXYZ，而非PointXYZRGB，否则报错
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source, 0, 0, 255);
	/* addPointCloud<>() 完成对颜色处理器对象的传递，为点云着色 */
	viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "sample cloud0002", v2);
	viewer->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud0002", v2);
	/*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");此两行可显示点云自带rgb颜色信息
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(cloud, "y");根据点云的某个字段进行上色。
	常见的字段有x，y,z,normal_x（X方向上的法线）normal_y （Y方向上的法线）normal_z （Z方向上的法线）,rgb （颜色）,curvature （曲率）*/
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud0002");//设置点大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source cloud0002");//设置点大小

	//-----视口3-----
	int v3(0);
	viewer->createViewPort(0, 0, 0.5, 0.5, v3);
	viewer->setBackgroundColor(0, 0.5, 0, v3);
	viewer->addText("third windows", 10, 10, "v3 text", v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(source, 0, 0, 255);//设置点云颜色（0到255取值法）。颜色着色处理器着色
	viewer->addPointCloud<pcl::PointXYZ>(source, color3, "cloud0003", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud0003");//设置点大小

	//-----视口4-----
	int v4(0);
	viewer->createViewPort(0.5, 0, 1, 0.5, v4);
	viewer->setBackgroundColor(0.0, 255, 255, v4);
	viewer->addText("fourth", 10, 10, "v4 text", v4);
	viewer->addPointCloud(target, "sample_cloud4", v4);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample_cloud4");//设置点云颜色（0到1取值法）
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud4");//设置点大小
	
	while (!viewer->wasStopped())//必选。防止运行完窗口闪一下就消失
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void visualize_pcd(pointcloud::Ptr &pcd_src, pointcloud::Ptr &pcd_tgt, pointcloud::Ptr &pcd_final)
//函数中类指针带&的，则后续用(viewer->createViewPort）箭头，若无&，则用点（viewer.createViewPort）！！！！！-------------------
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer_two"));
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer_test"));
	//将viewer定义为智能共享指针，保证该指针在整个程序全局使用。boost::shared_ptr全局指针。若非全局，可用上一行的方式。

	//--------创建两个显示窗口并设置背景颜色------------
	int v1(0);
	int v2(0);
	viewer->createViewPort(0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//--------设置背景颜色------------
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("source and target clouds", 10, 10, "v1 text", v1);
	viewer->addText("source and ransac clouds", 10, 10, "v2 text", v2);
	viewer->addCoordinateSystem(0.1);//添加坐标轴
	//-----------给点云添加颜色-------------------------
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 255, 0, 0);//红
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 0, 255, 0);//绿
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 255, 255, 0);//蓝
	//----------添加点云到显示窗口----------------------
	viewer->addPointCloud(pcd_src, src_h, "source cloud", v1);
	viewer->addPointCloud(pcd_tgt, tgt_h, "target cloud", v1);
	viewer->addPointCloud(pcd_tgt, tgt_h, "tgt cloud", v2);
	viewer->addPointCloud(pcd_final, final_h, "final cloud", v2);

	/*while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
}
