#pragma once
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <limits>
#include <string>
#include <boost/foreach.hpp>

class PlaceRecordConfig{
 public:
  double minx, maxx, miny, maxy, minz, maxz;
  double self_filter_radius;
  float icp_voxel_size;
  double sor_meank;  // 100
  double sor_stddev_multhresh;  // 1.0
  PlaceRecordConfig() {
    // default values
    minx = -std::numeric_limits<double>::infinity();
    maxx = std::numeric_limits<double>::infinity();
    miny = -std::numeric_limits<double>::infinity();
    maxy = std::numeric_limits<double>::infinity();
    minz = -std::numeric_limits<double>::infinity();
    maxz = std::numeric_limits<double>::infinity();
    self_filter_radius = 0.0;  // means do not apply this filter
    sor_meank = 100;
    sor_stddev_multhresh = 1.0;
    icp_voxel_size = 0.2;
  }
};

class PlaceRecord {
 protected:
  tf::StampedTransform pose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lowres_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_outliers;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
  boost::shared_ptr<pcl::SegmentDifferences<pcl::PointXYZ> > compare;
  bool tree_computed;
  void Setup(const PlaceRecordConfig& config);

 public:
  PlaceRecord(const tf::StampedTransform& pose_,
              const std::string& filename,
              const PlaceRecordConfig& config);
  PlaceRecord(const tf::StampedTransform& pose_,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
              const PlaceRecordConfig& config);
  PlaceRecord(const tf::StampedTransform& pose_,
              const sensor_msgs::PointCloud2& msg,
              const PlaceRecordConfig& config);
  const tf::StampedTransform& GetPose()const {return pose;}

  const boost::shared_ptr<pcl::SegmentDifferences<pcl::PointXYZ> >
    GetCompare(double dist_threshold);
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& GetFilteredCloud()const;
  //  tf::StampedTransform GetPose()const {return pose;}
  pcl::PointCloud<pcl::PointXYZ> GetFullCloud() const { return *cloud;}
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetFullCloudPtr() const { return cloud;}
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetLowResCloudPtr() const { return lowres_cloud;}
  pcl::PointCloud<pcl::PointXYZ> GetLowResCloud() const { return *lowres_cloud;}
};

