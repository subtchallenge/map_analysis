#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <map_analysis/viewers.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <utility>
#include <pcl/octree/octree_impl.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#define foreach BOOST_FOREACH
typedef pcl::PointXYZ POINT_T;

void
map_analysis_viewer(pcl::visualization::PCLVisualizer::Ptr& viewer,
                    const std::string& text,
                    const pcl::PointCloud<POINT_T>::Ptr& cloud,
                    const pcl::PointCloud<POINT_T>::Ptr& changestm,
                    const pcl::PointCloud<POINT_T>::Ptr& changesmt,
                    const pcl::PointCloud<POINT_T>::Ptr& model);

double getInliersInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& test,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& model,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& changes,
                         boost::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr&> inliers,
                         double dist_threshold = 1.0);

std::pair<double, double>
ComputeChanges(const pcl::PointCloud<pcl::PointXYZ>::Ptr& test,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr& model,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& changestm,
               boost::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr&> inlierstm,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& changesmt,
               boost::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr&> inliersmt,
               double inlier_eps=1.0,
               double outlier_eps=2.0);

std::string ExtractTeamname(const std::string& bagname);

void save_cloud(const std::string& outpcdfilename,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& temp_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& changestm,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& changesmt);

double datestr_to_epoch(const std::string& datestr);

