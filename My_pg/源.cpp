#include <fstream>
#include <math.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // ���ÿ���̨����ʱ��
#include <vtkAutoInit.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>//���������һ������׼
#include <pcl/filters/voxel_grid.h>//�����²����˲�
#include <pcl/features/normal_3d_omp.h>//ʹ��OMP��Ҫ��ӵ�ͷ�ļ�
#include <pcl/features/fpfh_omp.h> //fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/ia_ransac.h>//sac_ia�㷨
#include <boost/thread/thread.hpp>
#include"test.h"
//��׼Ŀ�ģ�������ͳһ��ͬһ������ϵ�£��ռ�����ת��������ƴ��
//˼�������Ʋ������������������������λ��ಹ��
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne1;//OpenMP���ٷ��߹��ƣ�OpenMPʹ�ö��/���߳������ټ���
	//Normalģ��Ϊ��������pcl::Normals normal���ĸ�ֵ��ʾ(normal_x, normal_y, normal_z, curvature);
	//������������X,Y,Z��ʾ�Ľ�������һ��Ӧ�������ʣ�curvature��
	ne1.setInputCloud(input_cloud);//�������
	ne1.setNumberOfThreads(8);// ����openMP���߳������ֶ������߳�����������ʾ����

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
	ne1.setSearchMethod(tree1);// ������ʽ
  // Output datasets������ݼ�
	pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>);//�ṩһ����ʼ�����Ʒ�����Normal
	//ne1.setRadiusSearch(0.03); // �����뾶
	ne1.setKSearch(10); // K���ڵ����
	ne1.compute(*normals_ptr); //  Compute the features���������������㷨��
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*input_cloud, *normals_ptr, *cloud_with_normals);//���������ĵ��Ʋ��洢
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
	//visualization01(input_cloud, normals_ptr);//���ӻ����з������ĵ���---------------------------------------���ӻ�------------

	cout << "0123" << endl;
	//pcl::PointCloud<pcl::PointNormal>::Ptr align_normals(new pcl::PointCloud<pcl::PointNormal>);//�ṩһ����ʼ������
	//pcl::concatenateFields(*input_cloud, *normals_ptr, *align_normals);//�����ֶΣ�cloud_with_normals�洢�������
	//-------------------------FPFH����-------------------------
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setNumberOfThreads(8);     //ָ��8�˼���
	fest.setInputCloud(input_cloud);//�������
	fest.setInputNormals(normals_ptr);  //���뷨��
	fest.setSearchMethod(tree1);     //������ʽ
	fest.setKSearch(10);            //K���ڵ����
	//fest.setRadiusSearch(0.025);  //�����뾶
	cout << "01234" << endl;
	fest.compute(*fpfh);            //����FPFH
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
	//---------------------���ص�������------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("rabit.pcd", *source);//ԭ����......
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabit.pcd", *source) < 0)
	{
		PCL_ERROR("->�����ļ������ڣ�\a\n");
		system("pause");
		return -1;
	}
	cout << "��Դ�����ж�ȡ " << source->size() << " ����" << endl;

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr source1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> grid;// �����˲�������-----
	grid.setInputCloud(source);
	Eigen::Vector4f leaf_size{ 0.01,0.01,0.01,0 }; // setLeafSize��������һ�ֳ�ʼ����ʽ
	grid.setLeafSize(leaf_size);
	grid.setMinimumPointsNumberPerVoxel(10);      // ����ÿһ����������Ҫ��������С�����
	grid.filter(*source1);
	cout << "�˲���Դ����: " << *source1 << endl;*/
	//--------------------------Ŀ�����----------
	pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bun0001.pcd", *tcloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bun0001.pcd", *tcloud) < 0)
	{
		PCL_ERROR("->�����ļ������ڣ�\a\n");
		system("pause");
		return -1;
	}
	cout << "��Ŀ������ж�ȡ " << tcloud->size() << " ����" << endl;

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

	cout << "�任����\n" << transform_1 << std::endl;

	// ִ�б任����������������´����� transformed_cloud ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*tcloud, *t_cloud, transform_1);
	cout << "��ת�������ж�ȡ " << t_cloud->size() << " ����" << endl;//Ŀ�����

	windows_2cloud(source, t_cloud);//������ʾ������----------------------------------------------------------------���ӻ�-----
	//my_ransac(source, t_cloud, align);//ransac����׼
	//visualize_pcd(source, t_cloud, align);//���ӻ����
	//---------------����Դ���ƺ�Ŀ����Ƶ�FPFH----------------------
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(t_cloud);

	//--------------------RANSAC������׼-----------------------------
	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> r_sac;
	r_sac.setInputSource(source);            // Դ����
	r_sac.setInputTarget(t_cloud);            // Ŀ�����
	r_sac.setSourceFeatures(source_fpfh);    // Դ����FPFH����
	r_sac.setTargetFeatures(target_fpfh);    // Ŀ�����FPFH����
	r_sac.setCorrespondenceRandomness(5);    // ��ѡ�����������Ӧʱ������Ҫʹ�õ��ھӵ�����,��ֵԽ������ƥ��������Խ��
	r_sac.setInlierFraction(0.1f);           // �����(�����)inlier����
	r_sac.setNumberOfSamples(8);             // ÿ�ε�����ʹ�õĲ���������
	r_sac.setSimilarityThreshold(0.1f);      // ���ײ����ζ�Ӧ�ܾ�������ı�Ե����֮���������ֵ����Ϊ[0,1]������1Ϊ��ȫƥ�䡣
	r_sac.setMaxCorrespondenceDistance(1.0f);// �ڵ㣬��ֵ Inlier threshold
	r_sac.setMaximumIterations(200);         // RANSAC ������������
	pointcloud::Ptr align(new pointcloud);
	r_sac.align(*align);
	pcl::io::savePCDFileASCII("align.pcd", *align);//�������------
	pcl::transformPointCloud(*source, *align, r_sac.getFinalTransformation());
	cout << "�任����\n" << r_sac.getFinalTransformation() << endl;
	windows_2cloud(source, align);//������ʾ������---------------------------------------------------------------���ӻ�-------
	//pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//*align_cloud = *t_cloud + *align;// �ϲ����ơ�ʹ������������ӵ���ʽ�������������ݽ��кϲ�(��ָ�룡������)
	//visualize_one_cloud(align_cloud);//���ӻ���������-----------------------------------------------------------���ӻ�----------
	visualize_pcd(source, t_cloud, align);//�����ڵ��ƶԱ�--------------------------------------------------------���ӻ�-------
	
	//--------------------��ʼ��ICP����--------------------
	pcl::console::TicToc time;
	time.tic();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//----------------------icp���Ĵ���--------------------
	icp.setInputSource(align);            // Դ����
	icp.setInputTarget(t_cloud);            // Ŀ�����
	icp.setTransformationEpsilon(1e-10);   // Ϊ��ֹ����������Сת������.ǰһ���任����͵�ǰ�任����Ĳ���С����ֵʱ������Ϊ�Ѿ������ˣ���һ������������
	icp.setMaxCorrespondenceDistance(1);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	icp.setEuclideanFitnessEpsilon(0.001);  // �������������Ǿ�������С����ֵ�� ֹͣ������������������������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(300);           // ����������
	icp.setUseReciprocalCorrespondences(true);//����Ϊtrue,��ʹ���໥��Ӧ��ϵ
	// ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//�洢���
	icp.align(*icp_cloud);//������׼������洢��icp_cloud��

	cout << "���� " << 300<< " ����ʱ " << time.toc() << " ms" << endl;//ʱ��
	cout << "\nICP has ����, ���� is " << icp.getFitnessScore() << endl;//FitnessScore��������?
	cout << "�任����\n" << icp.getFinalTransformation() << endl;
	
	pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());// ʹ�ô����ı任��Ϊ����Դ���ƽ��б任
	//pcl::io::savePCDFile("icp_cloud.pcd", *icp_cloud);
	cout << "��׼������" << icp_cloud->size() << endl;
	visualize_pcd(source, t_cloud, icp_cloud);//�����ڵ��ƶԱ�--------------------------------------------------------���ӻ�-------
	duiying_point(source, icp_cloud);//��ʾ��Ӧ�����߲�---------���ӻ�------------------
	return (0);
}
