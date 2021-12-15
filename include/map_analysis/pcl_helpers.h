#pragma once

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/passthrough.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr clean_robot(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_original_){

pcl::PointCloud<pcl::PointXYZ>::Ptr temp_;
pcl::copyPointCloud(temp_, point_cloud_original_);

 BOOST_FOREACH (const pcl::PointXYZRGB& pt, output.points){
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.r = 255;
    point.g = 0;
    point.b = 0;
    temp->push_back(point);
  }

return temp;

}



