#include <map_analysis/analysis.h>
#include <map_analysis/segment_differences_inliers.h>
//#include <map_analysis/segment_differences_inliers.hpp>
#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>

PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

void
map_analysis_viewer(pcl::visualization::PCLVisualizer::Ptr& viewer,
                    const std::string& text,
                    const pcl::PointCloud<POINT_T>::Ptr& cloud,
                    const pcl::PointCloud<POINT_T>::Ptr& changestm,
                    const pcl::PointCloud<POINT_T>::Ptr& changesmt,
                    const pcl::PointCloud<POINT_T>::Ptr& model) {
  std::string cloudname("cloud");
  std::string modelname("model");
  std::string changestmname("changestm");
  std::string changesmtname("changesmt");
  std::string textname("v0text");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_green(cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_red(changestm, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_blue(changesmt, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_white(model, 255, 255, 255);

  if (!viewer) {
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("ICP debug viewer"));
    viewer->initCameraParameters();
    int v1(0);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addText(text.c_str(), 10, 10, 10, 1.0, 1.0, 1.0, textname.c_str(), v1);
    viewer->addPointCloud<POINT_T>(model, single_color_white, modelname.c_str(), v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, modelname.c_str());

    viewer->addPointCloud<POINT_T> (cloud, single_color_green, cloudname.c_str(), v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudname.c_str());
    viewer->addPointCloud<POINT_T> (changestm, single_color_red, changestmname.c_str(), v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, changestmname.c_str());
    viewer->addPointCloud<POINT_T> (changesmt, single_color_blue, changesmtname.c_str(), v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, changesmtname.c_str());
  } else {
    // Update
    viewer->updatePointCloud<POINT_T>(cloud, single_color_green, cloudname.c_str());
    viewer->updatePointCloud<POINT_T>(changestm, single_color_red, changestmname.c_str());
    viewer->updatePointCloud<POINT_T>(changesmt, single_color_blue, changesmtname.c_str());
    viewer->updateText(text.c_str(), 10, 10, textname.c_str());
  }
  return;
}

double getInliersInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& test,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& model,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& changes,
                         boost::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr&> inliers,
                         double dist_threshold) {

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(model);
  boost::shared_ptr<pcl::SegmentDifferencesInliers<pcl::PointXYZ> > compare(new pcl::SegmentDifferencesInliers<pcl::PointXYZ>);
  compare->setInputCloud(model);
  compare->setSearchMethod(tree);
  compare->setDistanceThreshold(dist_threshold);
  compare->setTargetCloud(test);
  //ROS_INFO_STREAM("Setup complete");
  if (inliers != boost::none) {
    compare->segmentInliers(*changes, **inliers);
  } else {
    compare->segment(*changes);
  }
  //  std::cout << "Got " << changes->size() << " change points.  Test size is " << test->size() << " model size is " << model->size() << std::endl;

  return static_cast<double>(changes->size()) / static_cast<double>(model->size());
}

std::pair<double, double>
ComputeChanges(const pcl::PointCloud<pcl::PointXYZ>::Ptr& test,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr& model,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& changestm,
               boost::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr&> inlierstm,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& changesmt,
               boost::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr&> inliersmt,
               double inlier_eps,
               double outlier_eps) {
  static unsigned int count = 0;

  ros::WallTime init = ros::WallTime::now();
  double ttom = getInliersInCloud(test, model, changestm, inlierstm, inlier_eps);
  double mtot = getInliersInCloud(model, test, changesmt, inliersmt, outlier_eps);
  //  std::cout << "Got " << ttom << " test to model rate and " << mtot << " model to test rate " << std::endl;
  ros::WallTime final = ros::WallTime::now();
  // ROS_INFO_STREAM("Segmentation Took: " << (final - init).toSec());
  count++;
  return std::make_pair(1.0-ttom, mtot);
}
void save_cloud(const std::string& outpcdfilename,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& temp_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& changestm,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& changesmt) {
  if (outpcdfilename == std::string()) return;
  pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
  pcl::PointXYZRGB newpt;
  newpt.r = 255; newpt.g = 0; newpt.b = 0;
  for (auto& pt: changestm->points) {
    newpt.x = pt.x; newpt.y = pt.y; newpt.z = pt.z;
    out_cloud.push_back(newpt);
  }
  newpt.r = 0; newpt.g = 255; newpt.b = 0;
  for (auto& pt: temp_cloud->points) {
    newpt.x = pt.x; newpt.y = pt.y; newpt.z = pt.z;
    out_cloud.push_back(newpt);
  }
  newpt.r = 0; newpt.g = 0; newpt.b = 255;
  for (auto& pt: changesmt->points) {
    newpt.x = pt.x; newpt.y = pt.y; newpt.z = pt.z;
    out_cloud.push_back(newpt);
  }
  pcl::io::savePCDFile(outpcdfilename, out_cloud);
}

double datestr_to_epoch(const std::string& datestr) {
  // 2020-02-26 20:45:17.365757+00:00   example date string
  int year;
  int month;
  int day;
  int hour;
  int min;
  double sec;
  struct tm t;
  time_t t_of_day;
  std::vector <std::string> tokens;
  std::stringstream check1(datestr);
  std::string intermediate;
  // Tokenizing w.r.t. '-'
  getline(check1, intermediate, '-');
  year = atoi(intermediate.c_str());
  getline(check1, intermediate, '-');
  month = atoi(intermediate.c_str());
  getline(check1, intermediate, ' ');
  day = atoi(intermediate.c_str());

  getline(check1, intermediate, ':');
  hour = atoi(intermediate.c_str());
  getline(check1, intermediate, ':');
  min = atoi(intermediate.c_str());
  getline(check1, intermediate, '+');
  sec = atof(intermediate.c_str());

  t.tm_year = year-1900;  // Year - 1900
  t.tm_mon = month-1;           // Month, where 0 = jan. Assuming string specified with 1 = jan, hence -1
  t.tm_mday = day;          // Day of the month
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = floor(sec);
  t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);
  double rostime = t_of_day + (sec) - floor(sec) - 5 * 60 * 60;  // Converting from UTC to PST for urban, 9 hrs -
  return rostime;
}

