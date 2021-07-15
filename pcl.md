# For 

`PCD FILE`

```bash
# unorganized
[In] 
head table_scene_lms400.pcd 

[Out]
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity distance sid
SIZE 4 4 4 4 4 4
TYPE F F F F F F
COUNT 1 1 1 1 1 1
WIDTH 460400
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 460400

# organized: cloud->isOrganized()
[In] 
head table_scene_mug_stereo_textured.pcd 

[Out]
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgba
SIZE 4 4 4 4
TYPE F F F U
COUNT 1 1 1 1
WIDTH 640
HEIGHT 480
VIEWPOINT 0 0 0 0 1 0 0
POINTS 307200
```

`Check fields`

```c++
// fields: "rgb" "rgba"
template <typename T>
bool checkRGBFields(const std::string& fields)
{
    std::vector<pcl::PCLPointField> fields_;
    bool capable_=false;
    int field_idx_ = pcl::getFieldIndex<T> (fields, fields_);
    if (field_idx_ != -1)
    {
        capable_ = true;
    }
    return capable_;
}

//using 
bool capable=M_UTILITY::checkRGBFields<pcl::PointXYZ>("rgb");
std::cout<<capable<<'\n';
capable=M_UTILITY::checkRGBFields<pcl::PointXYZRGB>("rgb");
std::cout<<capable<<'\n';
```

```bash
# output
0
1
```

`viewer for PointXYZ and PointXYZRGB`

```c++
template <typename T>
typename pcl::visualization::PCLVisualizer::Ptr simpleViewer(typename pcl::PointCloud<T>::ConstPtr cloud,
                                                             pcl::PointCloud<pcl::Normal>::ConstPtr normals=nullptr)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    bool capable_=false;
    capable_= checkRGBFields<T>("rgb") || checkRGBFields<T>("rgba");
    if(!capable_) // no rgb fields
    {
        viewer->addPointCloud<T> (cloud, "sample cloud");
    }
    else
    {
        pcl::visualization::PointCloudColorHandlerRGBField<T> rgb(cloud);
        viewer->addPointCloud<T> (cloud, rgb, "sample cloud");
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    if(normals)
        viewer->addPointCloudNormals<T, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

// using: viewer cloud with rgb or without rgba
std::string filename="table_scene_mug_stereo_textured.pcd";
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
reader.read(filename, *cloud);
reader.read(filename, *cloud1);
pcl::visualization::PCLVisualizer::Ptr viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud);
viewer->spin();

pcl::visualization::PCLVisualizer::Ptr viewer2=M_UTILITY::simpleViewer<pcl::PointXYZ>(cloud1);
viewer2->spin();

// using: viewer cloud and normals
#include "viewer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

int main ()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer::Ptr viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud, normals);
    viewer->spin();
    return 0;
}
```



`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})
```



`IO`

```c++
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
reader.read<pcl::PointXYZ> (filename, *cloud);
// pcl::io::loadPCDFile (filename, *cloud);
pcl::PCDWriter writer;
writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false); // binary = false
// pcl::io::savePCDFileASCII (filename, *cloud);
```



`Transform`

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("1.pcd", *cloud) < 0)
	{
		PCL_ERROR("目标文件不存在！\n");
		return -1;
	}

	/// 方式1：Matrix4f
	// 创建矩阵对象transform_1，初始化为4×4单位阵
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// 定义旋转矩阵，绕z轴
	float theta = M_PI / 4;		// 旋转弧度
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);
	// 定义在x轴上的平移，2.5m
	transform_1(0, 3) = 2.5;
	// 打印平移、旋转矩阵
	std::cout << "方式1: 使用Matrix4f\n";
	std::cout << transform_1 << std::endl;

	/// 方式2：Affine3f
	// 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	// 定义在x轴上的平移，2.5m
	transform_2.translation() << 2.5, 0.0, 0.0;	// 三个数分别对应X轴、Y轴、Z轴方向上的平移
	// 定义旋转矩阵，绕z轴
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));	//同理，UnitX(),绕X轴；UnitY(),绕Y轴.
	// 打印平移、旋转矩阵
	std::cout << "\n方式2: 使用Affine3f\n";
	std::cout << transform_2.matrix() << std::endl;	//注意：不是transform_2

	/// 执行转换
	// transform_1 或者 transform_2 都可以实现相同的转换
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);	//注意：不是transform_2.matrix()

	// Visualization可视化
	std::cout << "点云颜色：\n";
	std::cout << "原始点云：白色\n" << "平移、旋转后点云：红色\n";
	pcl::visualization::PCLVisualizer viewer("矩阵转换实例");

	/// 定义点云RGB颜色
	// 原始点云附色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 255, 255);
	viewer.addPointCloud(cloud, cloud_color, "original_cloud");
	// 变换后点云附色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color(transformed_cloud, 250, 0, 0);
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);	//定义坐标系
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景颜色
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // 设置可视化窗口位置

	while (!viewer.wasStopped())
	{ // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
```





`Filter`

Extract the point cloud outside/inside the specified index

```c++
template <typename T>
void filterFromIndices(typename pcl::PointCloud<T>::Ptr cloud, pcl::PointIndices::ConstPtr indices)
{
    pcl::ExtractIndices<T> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    // set true to extract the point cloud outside the specified index
    // set false to extract the point cloud inside the specified index
    extract.setNegative(false);
    extract.filter(*cloud);
}

// using 
cloud_camera_->sampleAbovePlane();
std::vector<int> indices = cloud_camera_->getSampleIndices();
pcl::PointIndices::Ptr indices1(new pcl::PointIndices);
indices1->indices=indices;
M_UTILITY::filterFromIndices<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed(), indices1);
```

Remove NAN

```c++
template <typename T>
void removeNans(typename pcl::PointCloud<T>::Ptr cloud_processed_)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::removeNaNFromPointCloud(*cloud_processed_, *cloud_processed_, inliers->indices);
}
```



[Filter](https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html)

Removing outliers using a `StatisticalOutlierRemoval` filter

```c++
#include <pcl/filters/statistical_outlier_removal.h>

// Create the filtering object
/*
The number of neighbors to analyze for each point is set to 50, and the standard deviation multiplier to 1. What this means is that all points who have a distance larger than 1 standard deviation of the mean distance to the query point will be marked as outliers and removed.
*/
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (cloud);
sor.setMeanK (50);
sor.setStddevMulThresh (1.0);
sor.filter (*cloud_filtered);
```

```c++
// utility.h
namespace M_UTILITY{
    pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    
    template <typename T>
    typename pcl::PointCloud<T>::Ptr filter(typename pcl::PointCloud<T>::Ptr cloud)
    {
        pcl::StatisticalOutlierRemoval<T> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>);
        sor.filter (*cloud_filtered);
        return cloud_filtered;
    }
}
// utility.cpp
namespace M_UTILITY{
    pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }
}
// using test pcl.cpp
std::string filename="table_scene_lms400.pcd";

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
reader.read<pcl::PointXYZ> (filename, *cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1=M_UTILITY::filter<pcl::PointXYZ>(cloud);

pcl::visualization::PCLVisualizer::Ptr viewer = M_UTILITY::simpleVis(cloud);
viewer->spin();

pcl::visualization::PCLVisualizer::Ptr viewer1 = M_UTILITY::simpleVis(cloud1);
viewer1->spin();
```

```cmake
// using CMakeLists.txt
add_library(${PROJECT_NAME}_viewer src/viewer.cpp)
target_link_libraries(${PROJECT_NAME}_viewer
                      ${PCL_LIBRARIES})

add_executable(testpcl src/testpcl.cpp)
target_link_libraries(testpcl ${PCL_LIBRARIES} ${PROJECT_NAME}_viewer)
```

```bash
[ 98%] Linking CXX shared library /home/zhao/catkin_ws/devel/lib/libgpd_ros_viewer.so
[ 98%] Built target gpd_ros_viewer
Scanning dependencies of target testpcl
[ 98%] Building CXX object gpd_ros/CMakeFiles/testpcl.dir/src/testpcl.cpp.o
[100%] Linking CXX executable /home/zhao/catkin_ws/devel/lib/gpd_ros/testpcl
[100%] Built target testpcl
```



`Calculate surface normals`

[Normals](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html)

```c++
Linking CXX shared library /home/zhao/catkin_ws/devel/lib/libgpd_ros_viewer.so#include <pcl/features/normal_3d.h>

// ----------------------------------------------------------------
// -----Calculate surface normals with a search radius of 0.05-----
// ----------------------------------------------------------------
pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
ne.setInputCloud (point_cloud_ptr);
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
ne.setSearchMethod (tree);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
ne.setRadiusSearch (0.05);
ne.compute (*cloud_normals1);
```

`Normal Estimation Using Integral Images`

[Organized](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation_using_integral_images.html#normal-estimation-using-integral-images)

```c++
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
    
int 
main ()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    return 0;
}
```



`visualizer`

[Visualizer](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcl_visualizer.html?highlight=PCLVisualizer)

```c++
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVis (
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
```



`Other`

Acquire the max/min point

```c++
#include <pcl/common/common.h>

pcl::PointXYZRGBA min_pt_pcl, max_pt_pcl;
pcl::getMinMax3D(*cloud_camera_->getCloudProcessed(), min_pt_pcl, max_pt_pcl);
const Eigen::Vector3f min_pt = min_pt_pcl.getVector3fMap();
const Eigen::Vector3f max_pt = max_pt_pcl.getVector3fMap();
```





`GPD`

```c++
// cause the dimension between cloud_processed and camera_source un consistent
// reverseNormals will throw out of range in cloud_processed->at(i)
cloud_camera_->sampleAbovePlane();
std::vector<int> indices = cloud_camera_->getSampleIndices();
pcl::PointIndices::Ptr indices1(new pcl::PointIndices);
indices1->indices=indices;
// const PointCloudRGB::Ptr &getCloudProcessed() const
// template <typename T>
// void filterFromIndices(typename pcl::PointCloud<T>::Ptr cloud, pcl::PointIndices::ConstPtr indices)
// why it can be converted between const PointCloudRGB::Ptr and pcl::PointCloud<T>::Ptr???
M_UTILITY::filterFromIndices<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed(), indices1);
std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';
```

`C++`

常量指针与指针常量

**如果关键字`const`出现在星号左边，表示被指物是常量；如果出现在星号右边，表示指针自身是常量；如果出现在星号两边，表示被指物和指针两者都是常量**

```c++
//const int *ptr, int const *ptr, int * const ptr, const int * const ptr
const int* ptr1=new int(1);
int const* ptr2=new int(2);
int *const ptr3=new int(3);
const int *const ptr4=new int(4);
// error
// *ptr1=5;
// *ptr2=5;
// ptr3=ptr1;
// ok
// ptr1=ptr2;
// ptr2=ptr1;
// *ptr3=5;
```

`const` and `shared_ptr`

```c++
shared_ptr<T> p;             ---> T * p;                                    : nothing is const
const shared_ptr<T> p;       ---> T * const p;                              : p is const
shared_ptr<const T> p;       ---> const T * p;       <=> T const * p;       : *p is const
const shared_ptr<const T> p; ---> const T * const p; <=> T const * const p; : p and *p are const.
```

`ConstPtr` and `Ptr`

```c++
using Ptr = shared_ptr<PointCloud<PointT> >;
using ConstPtr = shared_ptr<const PointCloud<PointT> >;
```

Convert `Ptr` to `ConstPtr`

```c++
template <class T, class U>
  shared_ptr<T> const_pointer_cast (const shared_ptr<U>& sp) noexcept;

std::shared_ptr<int> foo;
std::shared_ptr<const int> bar;

foo = std::make_shared<int>(10);
bar = std::const_pointer_cast<const int>(foo);
```

Construct `ConstPtr` from `Ptr`

```cpp
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
std::cout<<ptr.use_count()<<'\n';
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cptr(ptr);
std::cout<<ptr.use_count()<<'\n';

// [out] 1 2
```

```cpp
// main ok
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::io::loadPCDFile (filename, *cloud);

pcl::visualization::PCLVisualizer::Ptr viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud);
viewer->spin();

// simpleViewer
// fields: "rgb" "rgba"
template <typename T>
bool checkRGBFields(const std::string& fields)
{
    std::vector<pcl::PCLPointField> fields_;
    bool capable_=false;
    int field_idx_ = pcl::getFieldIndex<T> (fields, fields_);
    if (field_idx_ != -1)
    {
        capable_ = true;
    }
    return capable_;
}

template <typename T>
typename pcl::visualization::PCLVisualizer::Ptr simpleViewer(typename pcl::PointCloud<T>::ConstPtr cloud,
                                                             pcl::PointCloud<pcl::Normal>::ConstPtr normals=nullptr)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    bool capable_=false;
    capable_= checkRGBFields<T>("rgb") || checkRGBFields<T>("rgba");
    if(!capable_) // no rgb fields
    {
        viewer->addPointCloud<T> (cloud, "sample cloud");
    }
    else
    {
        pcl::visualization::PointCloudColorHandlerRGBField<T> rgb(cloud);
        viewer->addPointCloud<T> (cloud, rgb, "sample cloud");
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    if(normals)
        viewer->addPointCloudNormals<T, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
```

`example`

```c++
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr glbptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

// It cannot be acheived if you want to encapsulate the Ptr object 
const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& get(){
    return glbptr;
}
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getConst(){
    return std::const_pointer_cast<const pcl::PointCloud<pcl::PointXYZRGBA> >(glbptr);
}

bool bvisualizer=true;

int main ()
{
    get()->points.resize(1); // ok to operate the object
    std::cout<<glbptr.use_count()<<'\n';
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr glbptr1=get();
    glbptr1->points.resize(2); // ok to operate the object
    std::cout<<glbptr.use_count()<<'\n';
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cglbptr2=getConst();
    std::cout<<glbptr.use_count()<<'\n';
    std::cout<<glbptr->points.size()<<'\n';
}
```

 ```bash
 # [output]
 1
 2
 3
 2
 ```

`trash`

```cpp
void removeNans(typename pcl::PointCloud<T>::Ptr cloud_processed_)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::removeNaNFromPointCloud(*cloud_processed_, *cloud_processed_, inliers->indices);
    // why????
    // !!! Process finished with exit code 139 (interrupted by signal 11: SIGSEGV) !!!
    //    pcl::removeNaNFromPointCloud(*cloud_processed_, inliers->indices);
    //    if (inliers->indices.size() < cloud_processed_->size()) {
    //        pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true);
    //        eifilter.setInputCloud(cloud_processed_);
    //        eifilter.setIndices(inliers);
    //        eifilter.filter(*cloud_processed_);
    //        printf("Cloud after removing NANs: %zu\n", cloud_processed_->size());
    //    }
}
```

