//
// Created by zhao on 2021/7/15.
//

#ifndef GPD_VIEWER_H
#define GPD_VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>

namespace M_UTILITY{

    template <typename T>
    void removeNans(typename pcl::PointCloud<T>::Ptr cloud_processed_)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::removeNaNFromPointCloud(*cloud_processed_, *cloud_processed_, inliers->indices);
    }

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

#endif //GPD_VIEWER_H
