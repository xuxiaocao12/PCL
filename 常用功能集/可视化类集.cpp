#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;
using namespace pcl;
/*--------------------
  -------help---------
  --------------------
*/
void printUsage(const char* progName)
{
	cout<<"\n\nUsage: "<<progName<<"[options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualisation example\n"
		<< "-r           RGB colour visualisation example\n"
		<< "-c           Custom colour visualisation example\n"
		<< "-n           Normals visualisation example\n"
		<< "-a           Shapes visualisation example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}

/*
 --------------------可视化单个点云：应用PCL Visualizer可视化类显示单个具有xyz信息的点云----------------
*/


//simplevis函数实现最基本的点云可视化操作
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	/*
	   open 3D viewer and add point cloud
	*/
		
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误。
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));

	//设置窗体的颜色
	viewer->setBackgroundColor(0, 0, 0);

	//将点云添加到视窗对象中
	/*
	  将点云添加到视窗对象中，并定义一个唯一的字符串作为ID号，
	  利用此字符串保证在其他成员中也能标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，每调用一次，就会创建一个新的ID号
	  如果想更新一个一已经显示的点云，必须先调用removePointCloud() ,并提供要更新的点云的ID号。
	*/
	viewer->addPointCloud<PointXYZ>(cloud, "sample cloud");

	//用以改变显示点云的尺寸，利用该方法可以控制点云在视窗中的显示方式。
		 /** \brief Set the rendering properties of a PointCloud 设置点云的呈现属性
		 * \param[in] property    the property type
		 * \param[in] value       the value to be set
		 * \param[in] id          the point cloud object id (default: cloud)
		 * \param[in] viewport    the view port where the Point Cloud's rendering properties should be modified (default: all)
		 */
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	/*
	  查看复杂的点云经常让人感觉没有方向感，为了保证正确的坐标判断，需要显示坐标系统方向，可以使用x(红色) Y（绿色） Z（蓝色）
	  圆柱体代表坐标轴的显示方式来解决。圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0
	*/
	viewer->addCoordinateSystem(1.0);

	//通过设置照相机参数，使得从默认的角度和方向观察点云
	viewer->initCameraParameters();
	return viewer;
}


/*
  ---------------可视化点云的颜色特征--------------------
  多数情况下，点云显示不采用简单的XYZ类型，常用的点云类型是XYZRGB点，包含颜色数据，除此之外，还可以给指定的点云定制颜色，以
  使得点云在视窗中比较容易区分。点赋予不同的颜色表征其对应的Z轴属性， PCL Visualizer可根据所存储的颜色数据为点云赋色，比如许多
  设备kinect可以获得带有RGB数据的点云， PCL Vizualizer可视化类可使用这种颜色数据为点云着色， rgbVis函数中的代码用于完成这种操作。
*/
//此处的点云使用带有RGB数据的属性字段
boost::shared_ptr<visualization::PCLVisualizer> rgbVis(PointCloud<PointXYZRGB>::ConstPtr cloud)
{
	/*
   open 3D viewer and add point cloud
   */
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);

	/*
	  设置窗口的背景颜色后。创建一个颜色处理对象， PointCloudColorHandlerRGBFiled利用这样的对象显示自定义颜色数据
	  PointCloudColorHandlerRGBField
		对象得到每个点云的RGB颜色字段，
	*/

	//PointCloudColorHandlerRGBField对象得到每个点云的RGB颜色字段，
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

/******************可视化点云自定义颜色特征**********************************************************/
//演示怎样给点云着上单独的一种颜色，可以利用该技术给指顶的点云单独着色，以区别其他的点云。

//点云类型为XYZ类型，customColourVis函数将点云赋值为绿色。

boost::shared_ptr<visualization::PCLVisualizer> customColourVis(PointCloud<PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//创建一个自定义的颜色处理器PointCloudHandlerCustom对象，并设置颜色为纯绿色
	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> single_color(cloud, 0, 255, 0);

	//addPointCloud<>() 完成对颜色处理器对象的传递
	viewer->addPointCloud<PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

/
//可视化点云法线和其他特征
/*
   显示法线是理解点云的一个重要的步骤，点云的法线特征是非常重要的基础特征，PCL Visualizer可视化类可用于绘制法线，
   也可以绘制表征点云的其他特征，比如主曲率和几何特征。
*/
//实现点云的法线

boost::shared_ptr<visualization::PCLVisualizer> normalsVis(PointCloud<PointXYZRGB>::ConstPtr cloud, PointCloud<pcl::Normal>::ConstPtr normals)
{
	// --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<PointXYZRGB>(cloud, rgb, "sample cloud");
	//设置点云的表现属性
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	//实现对点云法线的显示
	viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud,normals,10,0.05,"normals");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

/
//------------------------------------绘制普通形状----------------------------------------------------
/*
   PCL Visualizer可是实话类允许用户再视窗中绘制一般图元，这个类用于显示点云处理的可视化结果，例如，通过可视化球体
   包围聚类得到的点云集以显示据类结果。shapesVis函数用于实现添加形状到视窗中，添加了四种形状：从点云中的一个点到最后一个点之间的
   连线，原点所在的平面，以点云中的第一个点为中心的球体，沿Y轴的锥体。
*/

boost::shared_ptr<visualization::PCLVisualizer> shapesVis(PointCloud<PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<PointXYZRGB>(cloud, rgb, "sample cloud");
	//设置点云的表现属性
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	/*   绘制形状
	-----------------------绘制点之间的连线------------------------
	*/
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");
	//添加锥形参数
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return (viewer);
}


/* ---------------多视角显示---------------------
   PCL Visualizer可视化类允许用户通过不同的窗口（Viewport）绘制多个点云这样方便对点云比较
   下面的函数演示如何使用多视角来显示点云计算法线的方法结果对比。
*/

boost::shared_ptr<visualization::PCLVisualizer> viewportsVis(PointCloud<PointXYZRGB>::ConstPtr cloud,
	PointCloud<Normal>::ConstPtr normals1,
	PointCloud<Normal>::ConstPtr normals2)
{
	// --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();   //以上为创建视图的标准代码

	//创建新的视口
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //四个参数分别为x轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识。
	//设置视口的背景颜色
	viewer->setBackgroundColor(0, 0, 0, v1);
	//添加一个标签区别其他窗口， 利用RGB颜色着色器并添加点云到视口中。
	viewer->addText("Radius: 0.01",10,10,"v1 text",v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	//对第二视窗做同样的操作，使得创建的点云分布与有半窗口，将该视口背景赋值为灰色，虽然添加的是同样的点云，但是给点云自定义颜色来着色。
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
	//为所有视口设置属性，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
	//添加法线  每个视图都有一组对应的法线
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}

/*------------------处理鼠标事件的函数-----------------------
  每次相应鼠标时间，都会回电函数，需要从event实例提取时间信息，本例中查找鼠标左键的释放事件，
  每次响应这种事件都会再鼠标按下的位置生成一个文本标签。
*/
unsigned int text_id = 0;

void keyboardEventOccurred(const visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}
/********************************************************************************************
键盘事件 我们按下哪个按键  如果按下r健   则删除前面鼠标所产生的文本标签，需要注意的是，当按下R键时 3D相机仍然会重置
 所以在PCL中视窗中注册事件响应回调函数，不会覆盖其他成员对同一事件的响应
**************************************************************************************************/
void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

/******************自定义交互*****************************************************************************/
 /******************************************************************************************************
  多数情况下，默认的鼠标和键盘交互设置不能满足用户的需求，用户想扩展函数的某一些功能，  比如按下键盘时保存点云的信息，
  或者通过鼠标确定点云的位置   interactionCustomizationVis函数进行演示如何捕捉鼠标和键盘事件，在窗口点击，将会显示
  一个2D的文本标签，按下r健出去文本
  ******************************************************************************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//以上是实例化视窗的标准代码
	viewer->addCoordinateSystem(1.0);
	//分别注册响应键盘和鼠标事件，keyboardEventOccurred  mouseEventOccurred回调函数，需要将boost::shared_ptr强制转换为void*
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());

	return (viewer);
}

//主函数
int main(int argc, char** argv)
{
	// --------------------------------------
    // -----Parse Command Line Arguments----- 解析命令行参数
    // --------------------------------------

	//再参数列表argv中寻找“argument_name”给出的参数的位置
	   // 参数1.argc: 命令行参数的数量
	   // 参数2.argv: 命令行参数
	   // 参数3.argc: argument_name: 所要搜素的字符串
	//返回以找到的参数的索引，如果没有该参数，则返回-1。
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	bool simple(false), rgb(false), custom_c(false), normals(false),
		shapes(false), viewports(false), interaction_customization(false);
	if (pcl::console::find_argument(argc, argv, "-s") >= 0)
	{
		simple = true;
		std::cout << "Simple visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
	{
		custom_c = true;
		std::cout << "Custom colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
	{
		rgb = true;
		std::cout << "RGB colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-n") >= 0)
	{
		normals = true;
		std::cout << "Normals visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-a") >= 0)
	{
		shapes = true;
		std::cout << "Shapes visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-v") >= 0)
	{
		viewports = true;
		std::cout << "Viewports example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-i") >= 0)
	{
		interaction_customization = true;
		std::cout << "Interaction Customization example\n";
	}
	else
	{
		printUsage(argv[0]);
		return 0;
	}

	// ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
	PointCloud<PointXYZ>::Ptr basic_cloud_ptr(new PointCloud<PointXYZ>);
	PointCloud<PointXYZRGB>::Ptr point_cloud_ptr(new PointCloud<PointXYZRGB>);
	cout << "Genarating example point clouds.\n\n";
	// We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
	uint8_t r(255), g(15), b(15);

	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			PointXYZ basic_point;
			basic_point.x = 0.5*cos(pcl::deg2rad(angle));     //cosf 求弧度值的余弦值。针对float型的
			basic_point.y = sin(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;

			//使用位运算符将3个uint8_t 合并为一个uint32_t
			//有些数据类型中带有_t，_t表示这些数据类型是通过typedef定义的，而不是新的数据类型。
			//typedef unsigned int       uint32_t;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |                //static_cast是C++的一种转换运算符，用于强制隐式转换
				static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	//点云尺寸
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);


	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (simple)
	{
		viewer = simpleVis(basic_cloud_ptr);
	}
	else if (rgb)
	{
		viewer = rgbVis(point_cloud_ptr);
	}
	else if (custom_c)
	{
		viewer = customColourVis(basic_cloud_ptr);
	}
	else if (normals)
	{
		viewer = normalsVis(point_cloud_ptr, cloud_normals2);
	}
	else if (shapes)
	{
		viewer = shapesVis(point_cloud_ptr);
	}
	else if (viewports)
	{
		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
	}
	else if (interaction_customization)
	{
		viewer = interactionCustomizationVis();
	}

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
