#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <map_analysis/AnalyzePointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <utility>
#include <boost/random.hpp>

boost::shared_ptr<boost::mt19937> rng;
sensor_msgs::PointCloud2 PclToMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  return msg;
}
std::pair<float, float> RunTest(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  map_analysis::AnalyzePointCloud srv;
  srv.request.cloud = PclToMsg(cloud);
  ros::Time start = ros::Time::now();

  if (ros::service::call("/AnalyzeCloud", srv)) {
    ROS_ERROR_STREAM("Time taken by map analysis : "
                     << (ros::Time::now() - start).toSec()
                     << " seconds with a cloud of size " << cloud->points.size());
    return std::make_pair(srv.response.inlier_rate, srv.response.outlier_rate);
  }
  ROS_ERROR_STREAM("Call failed");
  return std::make_pair(-1.0f, -1.0f);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeRandomCloud(size_t num_points, float range = 100.0f) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ran_cld(new pcl::PointCloud<pcl::PointXYZ>);
  boost::random::uniform_real_distribution<float> distro(-range, range);
  for (size_t i = 0; i < num_points; i++) {
    pcl::PointXYZ pt;
    pt.x = distro(*rng);
    pt.y = distro(*rng);
    pt.z = distro(*rng);
    ran_cld->push_back(pt);
  }
  return ran_cld;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_map_analysis");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rng.reset(new boost::mt19937(floor(ros::Time::now().toSec()*1000)));


  ros::ServiceClient sc =
    nh.serviceClient<map_analysis::AnalyzePointCloud>("/AnalyzeCloud", true);
  ROS_ERROR_STREAM("Waiting to connect to analysis server");
  sc.waitForExistence();
  ROS_ERROR_STREAM("Connected to analysis server");
  // Here we are going to start by loading the same cloud as the model
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
  std::string modelfname;
  private_nh.param("model", modelfname, std::string("none"));
  PCL_INFO("Loading a pcd file %s\n", modelfname.c_str());
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (modelfname, *model) == -1) {
    PCL_ERROR("Couldn't read file %s\n", modelfname.c_str());
    exit(-1);
  }
  std::cout << "Loaded "
    << model->width * model->height
    << " points from" << modelfname
    << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *tmp_cloud += *model;
  while (false && tmp_cloud->points.size() < 6000000) {
    std::pair<float, float> test1 = RunTest(tmp_cloud);
    if (test1.first != 1.0) ROS_ERROR_STREAM("Expected 1.0 inlier rate on same cloud, got " << test1.first);
    if (test1.second != 0.0) ROS_ERROR_STREAM("Expected 0.0 inlier rate on same cloud, got " << test1.second);
    ros::Duration(1.0).sleep();
    *tmp_cloud += *model;  // Make bigger
  }
  size_t num_pts = 60;
  while (num_pts <= 6000000) {
    std::pair<float, float> test1 = RunTest(MakeRandomCloud(num_pts));
    ROS_ERROR_STREAM("Got inliers " << test1.first << " and outliers " << test1.second
                     << " on random test of size " << num_pts);
    ros::Duration(1.0).sleep();
    num_pts = num_pts * 10;
  }
  return -1;
}
