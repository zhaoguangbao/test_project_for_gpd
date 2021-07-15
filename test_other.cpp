//
// Created by zhao on 2021/7/15.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile("/home/zhao/catkin_ws/devel/lib/gpd_ros/table_scene_mug_stereo_textured.pcd", *cloud) < 0)
    {
        return -1;
    }
    // 1.0
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    float theta = M_PI / 4;
    transform_1(0, 0) = cos(theta);
    transform_1(0, 1) = -sin(theta);
    transform_1(1, 0) = sin(theta);
    transform_1(1, 1) = cos(theta);
    transform_1(0, 3) = 2.5;
    std::cout << transform_1 << std::endl;
    // 2.0
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    std::cout << transform_2.matrix() << std::endl;

    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

    pcl::visualization::PCLVisualizer viewer("transform");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_color, "original_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color(transformed_cloud, 250, 0, 0);
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color, "transformed_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400);

    viewer.spin();

    return 0;
}
