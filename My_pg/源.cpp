#include <fstream>
#include <math.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h> // icp算法
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // 利用控制台计算时间
#include <vtkAutoInit.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>//　随机采样一致性配准
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>//sac_ia算法
#include <boost/thread/thread.hpp>
#include"test.h"
//配准目的：将点云统一在同一个坐标系下，空间坐标转换，而非拼接
//思考：点云测量结果，三坐标测量结果，如何互相补偿
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne1;//OpenMP加速法线估计，OpenMP使用多核/多线程来加速计算
	//Normal模块为法向量，pcl::Normals normal的四个值表示(normal_x, normal_y, normal_z, curvature);
	//法向量用坐标X,Y,Z表示的结果，最后一个应该是曲率（curvature）
	ne1.setInputCloud(input_cloud);//输入点云
	ne1.setNumberOfThreads(8);// 设置openMP的线程数，手动设置线程数，否则提示错误

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
	ne1.setSearchMethod(tree1);// 搜索方式
  // Output datasets输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);//提供一个初始化点云法向量Normal
	//ne1.setRadiusSearch(0.03); // 搜索半径
	ne1.setKSearch(10); // K近邻点个数
	ne1.compute(*normals_ptr); //  Compute the features，计算特征，计算法线
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*input_cloud, *normals_ptr, *cloud_with_normals);//带法向量的点云并存储
	//pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);

	cout << "point normal size = " << normals_ptr->points.size() << endl;
	for (size_t i = 0; i < 2 && i < cloud_with_normals->size(); ++i)
	{
		cout << "{ " << normals_ptr->points[i].normal_x
			<< " " << cloud_with_normals->points[i].normal_y
			<< " " << cloud_with_normals->points[i].normal_z
			<< cloud_with_normals->points.at(i).curvature << "]"
			<< endl;
		cout << "try " << endl;
	}	
	//visualization01(input_cloud, normals_ptr);//可视化带有法向量的点云---------------------------------------可视化------------

	cout << "0123" << endl;
	//pcl::PointCloud<pcl::PointNormal>::Ptr align_normals(new pcl::PointCloud<pcl::PointNormal>);//提供一个初始化点云
	//pcl::concatenateFields(*input_cloud, *normals_ptr, *align_normals);//连接字段，cloud_with_normals存储有向点云
	//-------------------------FPFH估计-------------------------
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setNumberOfThreads(8);     //指定8核计算
	fest.setInputCloud(input_cloud);//输入点云
	fest.setInputNormals(normals_ptr);  //输入法线
	fest.setSearchMethod(tree1);     //搜索方式
	fest.setKSearch(10);            //K近邻点个数
	//fest.setRadiusSearch(0.025);  //搜索半径
	cout << "01234" << endl;
	fest.compute(*fpfh);            //计算FPFH
	for (int i = 0; i <2 ; i++) //fpfh->points.size()
	{
		pcl::FPFHSignature33 descriptor = fpfh->points[i];
		cout << descriptor << endl;
		cout << "test" << endl;
	}
	cout << "12345" << endl;
	return fpfh;
}
int
main(int argc, char** argv)
{
	//---------------------加载点云数据------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("rabit.pcd", *source);//原点云......
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabit.pcd", *source) < 0)
	{
		PCL_ERROR("->点云文件不存在！\a\n");
		system("pause");
		return -1;
	}
	cout << "从源点云中读取 " << source->size() << " 个点" << endl;

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr source1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;// 创建滤波器对象-----
	grid.setInputCloud(source);
	Eigen::Vector4f leaf_size{ 0.01,0.01,0.01,0 }; // setLeafSize参数的另一种初始化方式
	grid.setLeafSize(leaf_size);
	grid.setMinimumPointsNumberPerVoxel(10);      // 设置每一个体素内需要包含的最小点个数
	grid.filter(*source1);
	cout << "滤波后源点云: " << *source1 << endl;*/
	//--------------------------目标点云----------
	pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bun0001.pcd", *tcloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bun0001.pcd", *tcloud) < 0)
	{
		PCL_ERROR("->点云文件不存在！\a\n");
		system("pause");
		return -1;
	}
	cout << "从目标点云中读取 " << tcloud->size() << " 个点" << endl;

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	transform_1(0, 0) = 0.995870604090736;
	transform_1(0, 1) = 0.0461999293906251;
	transform_1(0, 2) = -0.0781492574005297;
	transform_1(0, 3) = 0.01;

	transform_1(1, 0) = -0.0170114840727817;
	transform_1(1, 1) = 0.940543863547451;
	transform_1(1, 2) = 0.339246002412522;
	transform_1(1, 3) = 0.01;

	transform_1(2, 0) = 0.0891759458463688;
	transform_1(2, 1) = -0.336515686510356;
	transform_1(2, 2) = 0.937445914927826;
	transform_1(2, 3) = 0.1;

	transform_1(3, 0) = 0.0;
	transform_1(3, 1) = 0.0;
	transform_1(3, 2) = 0.0;
	transform_1(3, 3) = 1.0;

	cout << "变换矩阵\n" << transform_1 << std::endl;

	// 执行变换，并将结果保存在新创建的 transformed_cloud 中
	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*tcloud, *t_cloud, transform_1);
	cout << "从转换点云中读取 " << t_cloud->size() << " 个点" << endl;//目标点云

	windows_2cloud(source, t_cloud);//单窗显示两点云----------------------------------------------------------------可视化-----
	//my_ransac(source, t_cloud, align);//ransac粗配准
	//visualize_pcd(source, t_cloud, align);//可视化结果
	//---------------计算源点云和目标点云的FPFH----------------------
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(t_cloud);

	//--------------------RANSAC点云配准-----------------------------
	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> r_sac;
	r_sac.setInputSource(source);            // 源点云
	r_sac.setInputTarget(t_cloud);            // 目标点云
	r_sac.setSourceFeatures(source_fpfh);    // 源点云FPFH特征
	r_sac.setTargetFeatures(target_fpfh);    // 目标点云FPFH特征
	r_sac.setCorrespondenceRandomness(5);    // 在选择随机特征对应时，设置要使用的邻居的数量,数值越大，特征匹配的随机性越大。
	r_sac.setInlierFraction(0.1f);           // 所需的(输入的)inlier分数
	r_sac.setNumberOfSamples(8);             // 每次迭代中使用的采样点数量
	r_sac.setSimilarityThreshold(0.1f);      // 将底层多边形对应拒绝器对象的边缘长度之间的相似阈值设置为[0,1]，其中1为完全匹配。
	r_sac.setMaxCorrespondenceDistance(1.0f);// 内点，阈值 Inlier threshold
	r_sac.setMaximumIterations(200);         // RANSAC 　最大迭代次数
	pointcloud::Ptr align(new pointcloud);
	r_sac.align(*align);
	pcl::io::savePCDFileASCII("align.pcd", *align);//保存点云------
	pcl::transformPointCloud(*source, *align, r_sac.getFinalTransformation());
	cout << "变换矩阵：\n" << r_sac.getFinalTransformation() << endl;
	windows_2cloud(source, align);//单窗显示两点云---------------------------------------------------------------可视化-------
	//pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//*align_cloud = *t_cloud + *align;// 合并点云。使用两个点云相加的形式对两个点云数据进行合并(加指针！！！！)
	//visualize_one_cloud(align_cloud);//可视化单个点云-----------------------------------------------------------可视化----------
	visualize_pcd(source, t_cloud, align);//两窗口点云对比--------------------------------------------------------可视化-------
	
	//--------------------初始化ICP对象--------------------
	pcl::console::TicToc time;
	time.tic();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//----------------------icp核心代码--------------------
	icp.setInputSource(align);            // 源点云
	icp.setInputTarget(t_cloud);            // 目标点云
	icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异.前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件；
	icp.setMaxCorrespondenceDistance(1);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(0.001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代。收敛条件，均方误差和小于阈值， 停止迭代。
	icp.setMaximumIterations(300);           // 最大迭代次数
	icp.setUseReciprocalCorrespondences(true);//设置为true,则使用相互对应关系
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//存储结果
	icp.align(*icp_cloud);//进行配准，结果存储在icp_cloud中

	cout << "迭代 " << 300<< " 次用时 " << time.toc() << " ms" << endl;//时间
	cout << "\nICP has 收敛, 分数 is " << icp.getFitnessScore() << endl;//FitnessScore误差均方根?
	cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
	
	pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());// 使用创建的变换对为输入源点云进行变换
	//pcl::io::savePCDFile("icp_cloud.pcd", *icp_cloud);
	cout << "配准点数：" << icp_cloud->size() << endl;
	visualize_pcd(source, t_cloud, icp_cloud);//两窗口点云对比--------------------------------------------------------可视化-------
	duiying_point(source, icp_cloud);//显示对应点连线并---------可视化------------------
	return (0);
}
