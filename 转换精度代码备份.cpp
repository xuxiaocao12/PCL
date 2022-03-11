#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <math.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>

void point1(double *x00, double *y00, double *z00);
void point2(double *x00, double *y00, double *z00);
void point3(double *x00, double *y00, double *z00);

using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;

void point1(double *x00, double *y00, double *z00)
{
	cout << "-----------开始处理点云数据1--------------" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//读入点云数据
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("GD1251210(1.8)-1.pcd", *cloud);
	cout << "点云数据1包含点数 " << cloud->points.size() << endl;
	cout << "加载中，请稍后" << endl;
	// -----------------统计滤波-------------------

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //设置待滤波的点云
	sor.setMeanK(800);           //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
	sor.filter(*cloud_filtered); //存储内点
	cout << "统计滤波后: \n" << cloud_filtered->points.size() << endl;
	
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(cloud_filtered);
	Eigen::Vector4f leaf_size{ 0.002,0.002,0.002,0 }; // setLeafSize参数的另一种初始化方式
	grid.setLeafSize(leaf_size);
	grid.setMinimumPointsNumberPerVoxel(10);      // 设置每一个体素内需要包含的最小点个数
	grid.filter(*cloud_filtered2);
	cout << "再体素滤波后: " << cloud_filtered2->points.size() << endl;
	
	//----------------------------最小二乘----------------------
	int num_points = cloud_filtered2->size();
	double X_avr, Y_avr, Z_avr;
	double  XX_avr, YY_avr, ZZ_avr;
	double  XY_avr, XZ_avr, YZ_avr;
	double  XXX_avr, YYY_avr, ZZZ_avr;
	double  XXY_avr, XXZ_avr, XYY_avr, XZZ_avr, YYZ_avr, YZZ_avr;

	//-------------average-xyz---------------
	double X = 0, Y = 0, Z = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		X += cloud_filtered2->points[i].x;
	}
	X_avr = X / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		Y += cloud_filtered2->points[i].y;
	}
	Y_avr = Y / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		Z += cloud_filtered2->points[i].z;
	}
	Z_avr = Z / num_points;
	//-------------average-xyz平方---------------
	double XX = 0, YY = 0, ZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XX += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x);
	}
	XX_avr = XX / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YY += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	YY_avr = YY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		ZZ += (cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	ZZ_avr = ZZ / num_points;
	//------------------------------------------
	double XY = 0, XZ = 0, YZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y);
	}
	XY_avr = XY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z);
	}
	XZ_avr = XZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z);
	}
	YZ_avr = YZ / num_points;
	//-------------average-xyz立方------------------------
	double XXX = 0, YYY = 0, ZZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXX += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x);
	}
	XXX_avr = XXX / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YYY += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	YYY_avr = YYY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		ZZZ += (cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	ZZZ_avr = ZZZ / num_points;
	//----------------------------------------------------
	double XXY = 0, XXZ = 0, XYY = 0, XZZ = 0, YYZ = 0, YZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y);
	}
	XXY_avr = XXY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z);
	}
	XXZ_avr = XXZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XYY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	XYY_avr = XYY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XZZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	XZZ_avr = XZZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YYZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z);
	}
	YYZ_avr = YYZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YZZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	YZZ_avr = YZZ / num_points;
	//------------最小二乘拟合球-------------------------------
	MatrixXd MatA(3, 3);//MatrixXd 代表 这个矩阵是double类型, X代表具体的行数都动态编译的
	MatA(0, 0) = XX_avr - X_avr * X_avr;
	MatA(0, 1) = XY_avr - X_avr * Y_avr;
	MatA(0, 2) = XZ_avr - X_avr * Z_avr;
	MatA(1, 0) = XY_avr - X_avr * Y_avr;
	MatA(1, 1) = YY_avr - Y_avr * Y_avr;
	MatA(1, 2) = YZ_avr - Y_avr * Z_avr;
	MatA(2, 0) = XZ_avr - X_avr * Z_avr;
	MatA(2, 1) = YZ_avr - Y_avr * Z_avr;
	MatA(2, 2) = ZZ_avr - Z_avr * Z_avr;

	VectorXd Vb(3);
	Vb(0) = 0.5*(XXX_avr - X_avr * XX_avr + XYY_avr - X_avr * YY_avr + XZZ_avr - X_avr * ZZ_avr);
	Vb(1) = 0.5*(XXY_avr - Y_avr * XX_avr + YYY_avr - Y_avr * YY_avr + YZZ_avr - Y_avr * ZZ_avr);
	Vb(2) = 0.5*(XXZ_avr - Z_avr * XX_avr + YYZ_avr - Z_avr * YY_avr + ZZZ_avr - Z_avr * ZZ_avr);
	MatrixXd MatA1 = MatA.inverse();
	VectorXd Vxyz = MatA1 * Vb;
	//cout << "MatA=\n" << MatA << endl;
	//cout << "MatA逆矩阵MatA1=\n" << MatA1 << endl;
	*x00 = Vxyz(0);
	*y00 = Vxyz(1);
	*z00 = Vxyz(2);
	cout << "圆心坐标:x=\n" << *x00 << "y=\n" << *y00 << "z=\n" << *z00 << endl;
	double R = sqrt(XX_avr - 2 * (*x00)*X_avr + (*x00) *(*x00) + YY_avr - 2 * (*y00)*Y_avr + (*y00)* (*y00) + ZZ_avr - 2 * (*z00)*Z_avr + (*z00) * (*z00));
	cout << "圆半径r=:\n" << R << endl;
}
void point2(double*x00, double*y00, double*z00)
{
	cout << "-----------开始处理点云数据2--------------" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//读入点云数据
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("GD1251210(1.8)-2.pcd", *cloud);
	cout << "点云数据2包含点数: " << cloud->points.size() << endl;
	cout << "加载中，请稍后" << endl;
	// -----------------统计滤波-------------------

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //设置待滤波的点云
	sor.setMeanK(800);           //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
	sor.filter(*cloud_filtered); //存储内点
	cout << "统计滤波后: \n" << cloud_filtered->points.size() << endl;

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(cloud_filtered);
	Eigen::Vector4f leaf_size{ 0.002,0.002,0.002,0 }; // setLeafSize参数的另一种初始化方式
	grid.setLeafSize(leaf_size);
	grid.setMinimumPointsNumberPerVoxel(10);      // 设置每一个体素内需要包含的最小点个数
	grid.filter(*cloud_filtered2);
	cout << "再体素滤波后: " << cloud_filtered2->points.size() << endl;
	//----------------------------最小二乘----------------------
	int num_points = cloud_filtered2->points.size();
	double X_avr, Y_avr, Z_avr;
	double  XX_avr, YY_avr, ZZ_avr;
	double  XY_avr, XZ_avr, YZ_avr;
	double  XXX_avr, YYY_avr, ZZZ_avr;
	double  XXY_avr, XXZ_avr, XYY_avr, XZZ_avr, YYZ_avr, YZZ_avr;

	//-------------average-xyz---------------
	double X = 0, Y = 0, Z = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		X += cloud_filtered2->points[i].x;
	}
	X_avr = X / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		Y += cloud_filtered2->points[i].y;
	}
	Y_avr = Y / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		Z += cloud_filtered2->points[i].z;
	}
	Z_avr = Z / num_points;
	//-------------average-xyz平方---------------
	double XX = 0, YY = 0, ZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XX += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x);
	}
	XX_avr = XX / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YY += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	YY_avr = YY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		ZZ += (cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	ZZ_avr = ZZ / num_points;
	//------------------------------------------
	double XY = 0, XZ = 0, YZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y);
	}
	XY_avr = XY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z);
	}
	XZ_avr = XZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z);
	}
	YZ_avr = YZ / num_points;
	//-------------average-xyz立方------------------------
	double XXX = 0, YYY = 0, ZZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXX += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x);
	}
	XXX_avr = XXX / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YYY += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	YYY_avr = YYY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		ZZZ += (cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	ZZZ_avr = ZZZ / num_points;
	//----------------------------------------------------
	double XXY = 0, XXZ = 0, XYY = 0, XZZ = 0, YYZ = 0, YZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y);
	}
	XXY_avr = XXY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z);
	}
	XXZ_avr = XXZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XYY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	XYY_avr = XYY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XZZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	XZZ_avr = XZZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YYZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z);
	}
	YYZ_avr = YYZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YZZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	YZZ_avr = YZZ / num_points;
	//------------最小二乘拟合球-------------------------------
	MatrixXd MatA(3, 3);//MatrixXd 代表 这个矩阵是double类型, X代表具体的行数都动态编译的
	MatA(0, 0) = XX_avr - X_avr * X_avr;
	MatA(0, 1) = XY_avr - X_avr * Y_avr;
	MatA(0, 2) = XZ_avr - X_avr * Z_avr;
	MatA(1, 0) = XY_avr - X_avr * Y_avr;
	MatA(1, 1) = YY_avr - Y_avr * Y_avr;
	MatA(1, 2) = YZ_avr - Y_avr * Z_avr;
	MatA(2, 0) = XZ_avr - X_avr * Z_avr;
	MatA(2, 1) = YZ_avr - Y_avr * Z_avr;
	MatA(2, 2) = ZZ_avr - Z_avr * Z_avr;

	VectorXd Vb(3);
	Vb(0) = 0.5*(XXX_avr - X_avr * XX_avr + XYY_avr - X_avr * YY_avr + XZZ_avr - X_avr * ZZ_avr);
	Vb(1) = 0.5*(XXY_avr - Y_avr * XX_avr + YYY_avr - Y_avr * YY_avr + YZZ_avr - Y_avr * ZZ_avr);
	Vb(2) = 0.5*(XXZ_avr - Z_avr * XX_avr + YYZ_avr - Z_avr * YY_avr + ZZZ_avr - Z_avr * ZZ_avr);
	MatrixXd MatA1 = MatA.inverse();//cout << "MatA逆矩阵MatA1=\n" << MatA1 << endl;
	VectorXd Vxyz = MatA1 * Vb;
	*x00 = Vxyz(0);
	*y00 = Vxyz(1);
	*z00 = Vxyz(2);
	cout << "圆心坐标:x=\t" << *x00 << "y=\t" << *y00 << "z=\t" << *z00 << endl;
	double R = sqrt(XX_avr - 2 * (*x00)*X_avr + (*x00) *(*x00) + YY_avr - 2 * (*y00)*Y_avr + (*y00)* (*y00) + ZZ_avr - 2 * (*z00)*Z_avr + (*z00) * (*z00));
	cout << "圆半径r=:\n" << R << endl;
}
void point3(double *x00, double *y00, double *z00)
{
	cout << "-----------开始处理点云数据3--------------" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//读入点云数据
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("GD1251210(1.8)-3.pcd", *cloud);
	cout << "点云数据3包含点数： " << cloud->points.size() << endl;
	cout << "加载中，请稍后" << endl;
	// -----------------统计滤波-------------------

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //设置待滤波的点云
	sor.setMeanK(800);           //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
	sor.filter(*cloud_filtered); //存储内点
	cout << "统计滤波后: \n" << cloud_filtered->points.size() << endl;
	
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(cloud_filtered);
	Eigen::Vector4f leaf_size{ 0.002,0.002,0.002,0 }; // setLeafSize参数的另一种初始化方式
	grid.setLeafSize(leaf_size);
	grid.setMinimumPointsNumberPerVoxel(10);      // 设置每一个体素内需要包含的最小点个数
	grid.filter(*cloud_filtered2);
	cout << "再体素滤波后: " << cloud_filtered2->points.size() << endl;

	//----------------------------最小二乘----------------------
	int num_points = cloud_filtered2->size();
	double X_avr, Y_avr, Z_avr;
	double  XX_avr, YY_avr, ZZ_avr;
	double  XY_avr, XZ_avr, YZ_avr;
	double  XXX_avr, YYY_avr, ZZZ_avr;
	double  XXY_avr, XXZ_avr, XYY_avr, XZZ_avr, YYZ_avr, YZZ_avr;

	//-------------average-xyz---------------
	double X = 0, Y = 0, Z = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		X += cloud_filtered2->points[i].x;
	}
	X_avr = X / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		Y += cloud_filtered2->points[i].y;
	}
	Y_avr = Y / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		Z += cloud_filtered2->points[i].z;
	}
	Z_avr = Z / num_points;
	//-------------average-xyz平方---------------
	double XX = 0, YY = 0, ZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XX += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x);
	}
	XX_avr = XX / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YY += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	YY_avr = YY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		ZZ += (cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	ZZ_avr = ZZ / num_points;
	//------------------------------------------
	double XY = 0, XZ = 0, YZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y);
	}
	XY_avr = XY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z);
	}
	XZ_avr = XZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z);
	}
	YZ_avr = YZ / num_points;
	//-------------average-xyz立方------------------------
	double XXX = 0, YYY = 0, ZZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXX += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x);
	}
	XXX_avr = XXX / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YYY += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	YYY_avr = YYY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		ZZZ += (cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	ZZZ_avr = ZZZ / num_points;
	//----------------------------------------------------
	double XXY = 0, XXZ = 0, XYY = 0, XZZ = 0, YYZ = 0, YZZ = 0;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y);
	}
	XXY_avr = XXY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XXZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z);
	}
	XXZ_avr = XXZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XYY += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y);
	}
	XYY_avr = XYY / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		XZZ += (cloud_filtered2->points[i].x)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	XZZ_avr = XZZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YYZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z);
	}
	YYZ_avr = YYZ / num_points;
	for (size_t i = 0; i < cloud_filtered2->points.size(); ++i)
	{
		YZZ += (cloud_filtered2->points[i].y)*(cloud_filtered2->points[i].z)*(cloud_filtered2->points[i].z);
	}
	YZZ_avr = YZZ / num_points;
	//------------最小二乘拟合球-------------------------------
	MatrixXd MatA(3, 3);//MatrixXd 代表 这个矩阵是double类型, X代表具体的行数都动态编译的
	MatA(0, 0) = XX_avr - X_avr * X_avr;
	MatA(0, 1) = XY_avr - X_avr * Y_avr;
	MatA(0, 2) = XZ_avr - X_avr * Z_avr;
	MatA(1, 0) = XY_avr - X_avr * Y_avr;
	MatA(1, 1) = YY_avr - Y_avr * Y_avr;
	MatA(1, 2) = YZ_avr - Y_avr * Z_avr;
	MatA(2, 0) = XZ_avr - X_avr * Z_avr;
	MatA(2, 1) = YZ_avr - Y_avr * Z_avr;
	MatA(2, 2) = ZZ_avr - Z_avr * Z_avr;

	VectorXd Vb(3);
	Vb(0) = 0.5*(XXX_avr - X_avr * XX_avr + XYY_avr - X_avr * YY_avr + XZZ_avr - X_avr * ZZ_avr);
	Vb(1) = 0.5*(XXY_avr - Y_avr * XX_avr + YYY_avr - Y_avr * YY_avr + YZZ_avr - Y_avr * ZZ_avr);
	Vb(2) = 0.5*(XXZ_avr - Z_avr * XX_avr + YYZ_avr - Z_avr * YY_avr + ZZZ_avr - Z_avr * ZZ_avr);
	MatrixXd MatA1 = MatA.inverse();//逆矩阵
	VectorXd Vxyz = MatA1 * Vb;
	*x00 = Vxyz(0);
	*y00 = Vxyz(1);
	*z00 = Vxyz(2);
	cout << "圆心坐标:x=\n" << *x00 << "y=\n" << *y00 << "z=\n" << *z00 << endl;
	double R = sqrt(XX_avr - 2 * (*x00)*X_avr + (*x00) *(*x00) + YY_avr - 2 * (*y00)*Y_avr + (*y00)* (*y00) + ZZ_avr - 2 * (*z00)*Z_avr + (*z00) * (*z00));
	cout << "圆半径r=:\n" << R << endl;
}
int
main(int argc, char* argv[])
{
	pcl::StopWatch time;
	double x10=0;
	double y10=0;
	double z10=0;
    point1(&x10, &y10, &z10);
	double x20 = 0;
	double y20 = 0;
	double z20 = 0;
	point2(&x20, &y20, &z20);
	double x30 = 0;
	double y30 = 0;
	double z30 = 0;
	point3(&x30, &y30, &z30);
	
	Matrix3d MatP;
	MatP << 1000 * x10, 1000 * y10, 1000 * z10,
		    1000 * x20, 1000 * y20, 1000 * z20,
		    1000 * x30, 1000 * y30, 1000 * z30;
	Matrix3d MatQ;
	MatQ << 504.712551, -406.783698, -475.952167,
		    579.659110, -401.343396, -346.077427,
	    	654.704553, -400.722006, -475.840503;
			cout << endl << endl << MatP << endl <<endl << MatQ << endl << endl;
			typedef Matrix< double, 1, 3> RowVector;//定义的行向量模板
			double up1 = (MatP(0, 0) + MatP(1, 0) + MatP(2, 0)) / 3;
			double up2 = (MatP(0, 1) + MatP(1, 1) + MatP(2, 1)) / 3;
			double up3 = (MatP(0, 2) + MatP(1, 2) + MatP(2, 2)) / 3;
			RowVector up(up1, up2, up3);
			double ux1 = (MatQ(0, 0) + MatQ(1, 0) + MatQ(2, 0)) / 3;
			double ux2 = (MatQ(0, 1) + MatQ(1, 1) + MatQ(2, 1)) / 3;
			double ux3 = (MatQ(0, 2) + MatQ(1, 2) + MatQ(2, 2)) / 3;
			RowVector ux(ux1, ux2, ux3);

			MatrixXd MatP_up(3, 3);
			for (int i = 0; i < 3; i++)
			{
				MatP_up(i, 0) = MatP(i, 0) - up1;
				MatP_up(i, 1) = MatP(i, 1) - up2;
				MatP_up(i, 2) = MatP(i, 2) - up3;
			}
			MatrixXd MatQ_ux(3, 3);
			for (int i = 0; i < 3; i++)
			{
				MatQ_ux(i, 0) = MatQ(i, 0) - ux1;
				MatQ_ux(i, 1) = MatQ(i, 1) - ux2;
				MatQ_ux(i, 2) = MatQ(i, 2) - ux3;
			}

			MatrixXd Cov = (MatP_up.transpose() * MatQ_ux) / 3;
			MatrixXd Cov_m = Cov - Cov.transpose();
			Matrix3d E;
			E << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			MatrixXd M = Cov + Cov.transpose() - Cov.trace()*E;
			MatrixXd Rq(4, 4);
			Rq << Cov.trace(), Cov_m(1, 2), Cov_m(2, 0), Cov_m(0, 1),
				Cov_m(1, 2), M(0, 0), M(0, 1), M(0, 2),
				Cov_m(2, 0), M(1, 0), M(1, 1), M(1, 2),
				Cov_m(0, 1), M(2, 0), M(2, 1), M(2, 2);

			EigenSolver<Matrix<double, 4, 4>> es(Rq);//求取矩阵特征值和特征向量的函数EigenSolver
			MatrixXcd evecs = es.eigenvectors();//获取矩阵特征向量4*4，这里定义的MatrixXcd必须有c，表示获得的是complex复数矩阵
			MatrixXcd evals = es.eigenvalues();//获取矩阵特征值 4*1
			MatrixXd evalsReal;//这里定义的MatrixXd里没有c
			evalsReal = evals.real();//获取特征值实数部分
			MatrixXd::Index evalsMax;//Index返回矩阵中最大值位置
			evalsReal.rowwise().sum().maxCoeff(&evalsMax);//得到最大特征值的位置。maxCoeff()，minCoeff()函数计算矩阵中最大值最小值
			Vector4d q;
			q << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax), evecs.real()(3, evalsMax);//对应最大特征向量
			double q0 = q(0);
			double q1 = q(1);
			double q2 = q(2);
			double q3 = q(3);
			Matrix3d Rd0;
			Rd0 << q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2 * (q1*q2 - q0 * q3), 2 * (q1*q3 + q0 * q2),
				2 * (q1*q2 + q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2 * (q2*q3 - q0 * q1),
				2 * (q1*q3 - q0 * q2), 2 * (q0*q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
			Matrix3d Rd;
			Rd << Rd0.transpose();//旋转矩阵
			RowVector Qr = ux - up * Rd;//平移向量
			//cout <<endl<< endl<<Qr << endl<<endl <<endl << endl  <<endl<<Rd << endl << endl;
			//--------------计算转换精度--------------
			Matrix3d p0;
			p0 << MatP * Rd;
			for (int i = 0; i < 3; i++)
			{
				p0(i, 0) = p0(i, 0) + Qr(0);
				p0(i, 1) = p0(i, 1) + Qr(1);
				p0(i, 2) = p0(i, 2) + Qr(2);
			}
			Matrix3d p;
			p << p0 - MatQ;//残差矩阵
			//------------标准差---------------------
			cout << p << endl << endl;
			double x = 0;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					x += p(i, j)*p(i, j);
				}
			}
			double k = 0;
			k = sqrt(x) / 3;
			cout << "坐标转换精度："<<k;
			cout << endl<<"程序运行时间:" << time.getTimeSeconds() << "秒" << endl;
	time.reset();
	return (0);
}
