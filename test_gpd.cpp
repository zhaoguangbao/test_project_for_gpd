#include "gpd/util/viewer.h"
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <gpd/util/cloud.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <memory>


int main ()
{
    bool bvisualizer=true;

    // load point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("/home/zhao/catkin_ws/devel/lib/gpd_ros/table_scene_mug_stereo_textured.pcd", *cloud);
    std::cout<<cloud->size()<<'\n';

    Eigen::Matrix3Xd view_points(3,1);
    view_points.col(0)<<0.0, 0.0, 0.0;

    pcl::visualization::PCLVisualizer::Ptr viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud);
    viewer->spin();

    gpd::util::Cloud* cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);

    cloud_camera_->removeNans();
    std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';

    pcl::PointXYZRGBA min_pt_pcl;
    pcl::PointXYZRGBA max_pt_pcl;
    pcl::getMinMax3D(*cloud_camera_->getCloudProcessed(), min_pt_pcl, max_pt_pcl);
    const Eigen::Vector3f min_pt = min_pt_pcl.getVector3fMap();
    const Eigen::Vector3f max_pt = max_pt_pcl.getVector3fMap();

    std::vector<double> workspace{-0.5, 0.8, -0.5, 0.2, 0.6, 1};
    cloud_camera_->filterWorkspace(workspace);
    std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';
    if(bvisualizer)
    {
        viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed());
        viewer->spin();
    }

    cloud_camera_->removeStatisticalOutliers();
    std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';
    if(bvisualizer)
    {
        viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed());
        viewer->spin();
    }

    // cause the dimension between cloud_processed and camera_source un consistent
    // reverseNormals will throw out of range in cloud_processed->at(i)
    cloud_camera_->sampleAbovePlane();
    std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';
    if(bvisualizer)
    {
        viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed());
        viewer->spin();
    }

    cloud_camera_->calculateNormals(8, 0.04);
    auto eigenNormals=cloud_camera_->getNormals();
    std::cout<<eigenNormals.size()<<'\n';

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->points.resize(eigenNormals.cols());
    for(int i=0;i<eigenNormals.cols();++i){
        pcl::Normal temp;
        temp.normal_x=eigenNormals(0, i);
        temp.normal_y=eigenNormals(1, i);
        temp.normal_z=eigenNormals(2, i);
        normals->points[i]=temp;
    }
    if(bvisualizer)
    {
        viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed(), normals);
        viewer->spin();
    }

    cloud_camera_->subsample(5000);
    std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';
    if(bvisualizer)
    {
        viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed());
        viewer->spin();
    }

//    pcl::io::savePCDFile("output.pcd", *cloud_camera_->getCloudProcessed());

//    cloud_camera_->voxelizeCloud(1);
//    std::cout<<cloud_camera_->getCloudProcessed()->size()<<'\n';
//    viewer=M_UTILITY::simpleViewer<pcl::PointXYZRGBA>(cloud_camera_->getCloudProcessed());
//    viewer->spin();



    /*
     * cloud.filterWorkspace(params_.workspace_);

  if (params_.voxelize_) {
    cloud.voxelizeCloud(params_.voxel_size_);
  }

  cloud.calculateNormals(params_.num_threads_, params_.normals_radius_);

  if (params_.refine_normals_k_ > 0) {
    cloud.refineNormals(params_.refine_normals_k_);
  }

  if (params_.sample_above_plane_) {
    cloud.sampleAbovePlane();
  }

  cloud.subsample(params_.num_samples_);
     * */

    return 0;
}