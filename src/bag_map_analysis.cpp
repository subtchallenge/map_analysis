#include <map_analysis/analysis.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <map_analysis/MapAnalyzerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <map_analysis/relative_metric.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <map_analysis/AnalyzePointCloud.h>
#include <map_analysis/ArtifactReport.h>
#include <map_analysis/RunStatus.h>
#include <map_analysis/MapMetrics.h>
#include <map_analysis/ArtifactMetrics.h>
#include <pcl/filters/crop_box.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// Mesh import stuff
#include <map_analysis/mesh.h>

double getRMSE(const std::set<double> &residuals) {
  double MSE = 0.0;

  for (std::set<double>::const_iterator itr = residuals.begin();
       itr != residuals.end(); itr++) {
    MSE += *itr * *itr;
  }
  MSE = MSE / static_cast<double>(residuals.size());
  return sqrt(MSE);
}


class LiveMapAnalyzer {
 protected:
  std::string modelfname;
  std::string meshfname;
  pcl::PointCloud<POINT_T>::Ptr model_;
  bool enabled;
  bool in_run;
  ros::Time max_duration;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter_;
  pcl::VoxelGrid<pcl::PCLPointCloud2> accumulate_filter_;
  ros::Subscriber cloud_sub;
  ros::Subscriber artifact_sub;
  ros::Subscriber run_status_sub;
  ros::Publisher inlier_cloud_pub;
  ros::Publisher outlier_cloud_pub;
  ros::Publisher full_cloud_pub;
  ros::Publisher accumulated_cloud_pub;
  ros::Publisher prop_marker_pub;
  ros::Publisher map_metrics_pub;
  ros::Publisher artifact_metrics_pub;
  dynamic_reconfigure::Server<map_analysis::MapAnalyzerConfig> server;
  map_analysis::MapAnalyzerConfig config_;
  std::vector<std::pair<std::string, tf::Pose> > gt_artifacts;

  // Mesh analysis loader / visualizer / checker
  RefMesh gtMesh_;

  ros::Timer subscribe_maps_timer;
  ros::Timer subscribe_maps_size_timer;
  ros::Timer evaluate_timer;

  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void ProcessClouds(const ros::TimerEvent& evt);
  pcl::PointCloud<pcl::PointXYZ>::Ptr UnionClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_old,
                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_new);
  bool AnalyzeServiceCb(map_analysis::AnalyzePointCloud::Request& req,
                        map_analysis::AnalyzePointCloud::Response& rsp);
  void AnalyzeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void ArtifactCallback(const map_analysis::ArtifactReport::ConstPtr& msg);
  void DoRelativeAssignment();
  void PublishCloud(ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void reconfigure_callback(map_analysis::MapAnalyzerConfig &config, uint32_t level);
  void PublishPropMarkers();
  void PublishJskVisuals();
  void LoadGTArtifacts(const std::string& fname);
  std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> relative_reports;
  std::vector<std::tuple<std::string, tf::Point, int, std::string, size_t>> good_artifacts;
  std::vector<std::tuple<std::string, tf::Point, int, std::string, size_t>> bad_artifacts;
  std::vector<std::tuple<tf::Point, int>> relative_frame_aligned_artifacts;
  std::set<double> residuals;
  std::set<double> residualsxy;
  std::map<std::string, ros::Subscriber> mr_subs;
  pcl::PointCloud<pcl::PointXYZ>::Ptr master_cloud;

  //
  // Global values to send to display
  double map_coverage;
  double map_outliers;
  size_t cloud_size;
  int total_cloud_size_;
  int acc_cloud_size_est_;
  double total_pre_ds_size_;
  double total_post_ds_size_;
  
  bool pub_once;

  double fps;
  int msg_sequence;

  // Mesh values for display
  double mesh_density;
  double mesh_face_coverage;

  double min_error;
  double max_error;
  double min_errorxy;
  double max_errorxy;
  size_t points;
  size_t pointsxy;
  double relative_RMSE;
  double RMSE;
  double RMSExy;
  double residual;
  double residualxy;
  double artifact_threshold;

  Eigen::Vector4f crop_box_min_pt;
  Eigen::Vector4f crop_box_max_pt;
  bool crop_starting_area;
  pcl::CropBox<pcl::PointXYZ> cropper;
  bool enable_mesh_analysis_;
  // Frame rate stuff
  //
  std::list<ros::Time> frame_history;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server;

  std::map<size_t, interactive_markers::MenuHandler> menu_handler;
  std::map<size_t, interactive_markers::MenuHandler::EntryHandle> h_mode_last;
  std::map<size_t, size_t> inlier_interactive_index;


  void modeCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, size_t ind);

  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg,
                                     const geometry_msgs::Point& pt);


  visualization_msgs::InteractiveMarker makeEmptyMarker(const geometry_msgs::Point& pt);

  void makeMenuMarker(size_t ind, const geometry_msgs::Point& pt);

  void initMenu(size_t ind);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void subscribeAvailableMaps(const ros::TimerEvent& event);
  void subscribeAvailableMapSizes(const ros::TimerEvent& event);
  ros::ServiceServer test_service;

 public:
  LiveMapAnalyzer();
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  
  void RunStatusCallback(const map_analysis::RunStatus::ConstPtr& msg);
  void RunStringCallback(const std_msgs::String::ConstPtr& msg);
  void MRCloudCallback(const std::string& robotname, const sensor_msgs::PointCloud2::ConstPtr& msg);
  void MRCloudRateCallback(const std::string& robotname, const std_msgs::Int32::ConstPtr& msg);
  void ProcessClouds();
  bool InRun();
  void ConfigInit(bool accumulate_mode, double inlier_tolerance, double outlier_tolerance, double min_leaf_size, double incremental_inlier_tolerance);
  
  rosbag::Bag bag_out;
  ros::Time current_time;
  double start_time;
  
  std::map<std::string, int> mr_cloud_sizes;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> mr_clouds;
};

bool LiveMapAnalyzer::InRun() {
  return in_run;
}

void LiveMapAnalyzer::
modeCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, size_t ind) {
  ROS_INFO_STREAM("In mode callback!" << ind);
  interactive_markers::MenuHandler::CheckState state;
  menu_handler[ind].getCheckState(h_mode_last[ind], state);
  if (state == interactive_markers::MenuHandler::CHECKED) {
    ROS_INFO_STREAM("Deactivating outlier report " << ind);
    menu_handler[ind].setCheckState(h_mode_last[ind], interactive_markers::MenuHandler::UNCHECKED);
  } else {
    ROS_INFO_STREAM("Activating outlier report " << ind);
    menu_handler[ind].setCheckState(h_mode_last[ind], interactive_markers::MenuHandler::CHECKED);
  }
  menu_handler[ind].reApply(*marker_server);
  marker_server->applyChanges();
  DoRelativeAssignment();
}



visualization_msgs::Marker LiveMapAnalyzer::
makeBox(visualization_msgs::InteractiveMarker &msg, const geometry_msgs::Point& pt) {
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.pose.position = pt;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.01;

  return marker;
}

visualization_msgs::InteractiveMarker LiveMapAnalyzer::
makeEmptyMarker(const geometry_msgs::Point& pt) {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "darpa";
  int_marker.pose.position = pt;
  int_marker.scale = 1;

  return int_marker;
}

void LiveMapAnalyzer::
makeMenuMarker(size_t ind_in, const geometry_msgs::Point& pt) {
  if (menu_handler.find(ind_in) != menu_handler.end()) {
    return;
  }
  std::stringstream namess;
  namess << "menu" << ind_in;
  initMenu(ind_in);
  visualization_msgs::InteractiveMarker int_marker = makeEmptyMarker(pt);
  int_marker.name = namess.str();

  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.name = "Menu";
  geometry_msgs::Point pt2;
  pt2.x = 0; pt2.y = 0; pt2.z = 0;
  control.markers.push_back(makeBox(int_marker, pt2));
  int_marker.controls.push_back(control);
  ROS_INFO_STREAM("Created interactive marker index " << ind_in);
  marker_server->insert(int_marker);
  marker_server->setCallback(int_marker.name, boost::bind(&LiveMapAnalyzer::processFeedback, this, _1));
  menu_handler[ind_in].apply(*marker_server, namess.str());
  marker_server->applyChanges();
}
void LiveMapAnalyzer::
processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM(s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      break;
  }

  marker_server->applyChanges();
}

void LiveMapAnalyzer::
initMenu(size_t ind) {
  interactive_markers::MenuHandler::EntryHandle h_first_entry;
  menu_handler[ind] = interactive_markers::MenuHandler();
  h_first_entry = menu_handler[ind].insert("Inlier?", boost::bind(&LiveMapAnalyzer::modeCb, this, _1, ind));
  menu_handler[ind].setCheckState(h_first_entry, interactive_markers::MenuHandler::CHECKED);
  h_mode_last[ind] = h_first_entry;
}

LiveMapAnalyzer::LiveMapAnalyzer(): nh(), private_nh("~") {
  min_error = std::numeric_limits<double>::infinity();
  max_error = 0.0;
  min_errorxy = std::numeric_limits<double>::infinity();
  max_errorxy = 0.0;
  points = 0;
  pointsxy = 0;
  relative_RMSE = 0.0;
  RMSE = 0.0;
  RMSExy = 0.0;
  residual = 0.0;
  residualxy = 0.0;
  map_coverage = 0.0;
  map_outliers = 0.0;
  mesh_face_coverage = 0.0;
  mesh_density = 0.0;
  cloud_size = 0;
  fps = 0.0;
  msg_sequence = 0;
  enabled = true;
  in_run = true;
  pub_once = false;
  total_cloud_size_ = 0;
  acc_cloud_size_est_ = 0;
  total_pre_ds_size_ = 0;
  total_post_ds_size_ = 0;
  private_nh.param("start_enabled", enabled, true);
  private_nh.param("model", modelfname, std::string("none"));
  private_nh.param("artifact_threshold", artifact_threshold, 5.0);
  private_nh.param("crop_starting_area", crop_starting_area, false);
  if (crop_starting_area) {
    std::vector<double> min_pt_vec, max_pt_vec;
    if (!private_nh.hasParam("crop_min_pt") || !private_nh.hasParam("crop_max_pt")) {
      crop_starting_area = false;
      ROS_ERROR_STREAM("ERROR!!!! Crop starting area was set but no bounds given in crop_min_pt and crop_max_pt!!!! Cropping disabled!");
    } else {
      ROS_INFO_STREAM("Getting crop bounds");
      private_nh.getParam("crop_min_pt", min_pt_vec);
      private_nh.getParam("crop_max_pt", max_pt_vec);
      ROS_INFO_STREAM("Params gotten");
      for (double d : min_pt_vec) {
        ROS_INFO_STREAM("d: "<< d);
      }
      crop_box_min_pt << min_pt_vec[0], min_pt_vec[1], min_pt_vec[2], 1.0;
      crop_box_max_pt << max_pt_vec[0], max_pt_vec[1], max_pt_vec[2], 1.0;
      cropper.setMin(crop_box_min_pt);
      cropper.setMax(crop_box_max_pt);
      cropper.setNegative(true);  // Remove points inside of box instead
      ROS_INFO_STREAM("Got crop bounds");
    }
  }
  enable_mesh_analysis_ = private_nh.getParam("mesh_model", meshfname); // if mesh model is provided, perform mesh-based analysis [EXPERIMENTAL]
  double analysis_update_interval;
  private_nh.param("analysis_update_interval", analysis_update_interval, 1.0);
  model_.reset(new pcl::PointCloud<POINT_T>);
  PCL_INFO("Loading a pcd file %s\n", modelfname.c_str());
  if (pcl::io::loadPCDFile<POINT_T> (modelfname, *model_) == -1) {
    PCL_ERROR("Couldn't read file %s\n - Defaulting to SubTLogo.pcd", modelfname.c_str());
    std::stringstream ss;
    std::string path = ros::package::getPath("map_analysis");
    ss << path << "/test/SubTLogo.pcd";
    if (pcl::io::loadPCDFile<POINT_T> (ss.str(), *model_) == -1) {
      PCL_ERROR("Couldn't read default pcd file %s\n - Exiting", ss.str());
      exit(-1);
    }
  }
  std::cout << "Loaded "
    << model_->width * model_->height
    << " points from" << modelfname
    << std::endl;
  if (crop_starting_area) {
    cropper.setInputCloud(model_);
    cropper.filter(*model_);
    ROS_INFO_STREAM("Model point cloud size after cropping starting area: " << model_->width*model_->height);
  }
  inlier_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("MA_inlier_cloud", 1, true);
  outlier_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("MA_outlier_cloud", 1, true);
  full_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("MA_full_cloud", 1, true);
  accumulated_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_cloud", 1, true);
  
  // Let's go ahead and publish the model cloud
  //PublishCloud(full_cloud_pub, model_);

  marker_server.reset(new interactive_markers::InteractiveMarkerServer("menu"));
  if (private_nh.hasParam("gt_artifact_filename")) {
    std::string gt_artifact_filename;
    private_nh.param("gt_artifact_filename", gt_artifact_filename, std::string("No artifacts"));
    LoadGTArtifacts(gt_artifact_filename);
  }
  map_metrics_pub = nh.advertise<map_analysis::MapMetrics>("map_metrics", 10, true);
  artifact_metrics_pub = nh.advertise<map_analysis::ArtifactMetrics>("artifact_metrics", 10, true);
  prop_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("props", 1, true);
  server.setCallback(boost::bind(&LiveMapAnalyzer::reconfigure_callback, this, _1, _2));
  if(enable_mesh_analysis_)
    gtMesh_.Process(meshfname);
  artifact_sub =
    nh.subscribe<map_analysis::ArtifactReport>("subt/artifact_reports", 10,
                                            boost::bind(&LiveMapAnalyzer::ArtifactCallback, this, _1));
  run_status_sub =
    nh.subscribe<map_analysis::RunStatus>("/subt/status", 10,
                                       boost::bind(&LiveMapAnalyzer::RunStatusCallback, this, _1));
  cloud_sub =
    nh.subscribe<sensor_msgs::PointCloud2>("cloud",
                                           1,
                                           boost::bind(&LiveMapAnalyzer::CloudCallback, this, _1));
  subscribe_maps_timer = nh.createTimer(ros::Duration(10.0), boost::bind(&LiveMapAnalyzer::subscribeAvailableMaps, this, _1));
  subscribe_maps_size_timer = nh.createTimer(ros::Duration(10.0), boost::bind(&LiveMapAnalyzer::subscribeAvailableMapSizes, this, _1));
  evaluate_timer = nh.createTimer(ros::Duration(analysis_update_interval), boost::bind(&LiveMapAnalyzer::ProcessClouds, this, _1));
  test_service =
    nh.advertiseService<map_analysis::AnalyzePointCloud::Request,
    map_analysis::AnalyzePointCloud::Response>("AnalyzeCloud",
                                               boost::bind(&LiveMapAnalyzer::AnalyzeServiceCb, this, _1, _2));
  geometry_msgs::Point pt;
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = 0.0;
  makeMenuMarker(0, pt);
}
// Once per second or 10 or so, grab all ros topics and look for ones with the /clouds namespace. These are multirobot clouds
//
void LiveMapAnalyzer::subscribeAvailableMaps(const ros::TimerEvent& event) {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.name.rfind("/clouds/", 0) == 0) {  // this topic starts with /clouds/ so it is one we will subscribe to
      if (mr_subs.find(info.name) == mr_subs.end()) {
        std::string topicname = info.name;
        std::string cloudname = info.name;
        cloudname.erase(0,8);
        mr_subs[info.name] = nh.subscribe<sensor_msgs::PointCloud2>(topicname, 1, boost::bind(&LiveMapAnalyzer::MRCloudCallback, this, cloudname, _1));
      }
    }
  }
}
// Once per second or 10 or so, grab all ros topics and look for ones with the /cloud_rates namespace. These are multirobot cloud rates
//
void LiveMapAnalyzer::subscribeAvailableMapSizes(const ros::TimerEvent& event) {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    const ros::master::TopicInfo& info = *it;
    if (info.name.rfind("/cloud_initial_size/", 0) == 0) {  // this topic starts with /cloud_initial_sizes/ so it is one we will subscribe to
      if (mr_subs.find(info.name) == mr_subs.end()) {
        std::string topicname = info.name;
        std::string cloudname = info.name;
        cloudname.erase(0,21);
        mr_subs[info.name] = nh.subscribe<std_msgs::Int32>(topicname, 1, boost::bind(&LiveMapAnalyzer::MRCloudRateCallback, this, cloudname, _1));
        mr_cloud_sizes[cloudname] = 0;
      }
    }
  }
}

void LiveMapAnalyzer::LoadGTArtifacts(const std::string& fname) {
  std::ifstream infile(fname.c_str(), std::ios::in);
  if (!infile.is_open()) {
    ROS_ERROR_STREAM("Please provide a valid env_gt filename. Unable to open: " << fname <<
                     "\nReading fallback artifacts.csv file.");
    std::stringstream ss;
    std::string path = ros::package::getPath("map_analysis");
    ss << path << "/test/artifacts.csv";
    infile.open(ss.str());
    if (!infile.is_open()) {
      ROS_ERROR_STREAM("Could not read file: " << ss.str() << ". Exiting.");
      exit(1);
    }
  }
  std::string line;
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    std::string label;
    double x, y, z;
    ss >> label >> x >> y >> z;
    tf::Pose tmp_pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, z));  
    gt_artifacts.push_back(std::make_pair(label, tmp_pose));
  }
  ROS_INFO_STREAM("Loaded " << gt_artifacts.size() << " ground truth artifacts");
  infile.close();
}

void LiveMapAnalyzer::reconfigure_callback(map_analysis::MapAnalyzerConfig &config, uint32_t level) {
  config_ = config;
}

void LiveMapAnalyzer::ConfigInit(bool accumulate_mode, double inlier_tolerance, double outlier_tolerance, double min_leaf_size, double incremental_inlier_tolerance)
{
  config_.accumulate_mode = accumulate_mode;
  config_.inlier_tolerance = inlier_tolerance;
  config_.outlier_tolerance = outlier_tolerance;
  config_.min_leaf_size = min_leaf_size;
  config_.incremental_inlier_tolerance = incremental_inlier_tolerance;
}

void LiveMapAnalyzer::PublishPropMarkers() {
  visualization_msgs::MarkerArray markerarray;
  visualization_msgs::Marker marker;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.header.frame_id = "darpa";
  marker.header.stamp = ros::Time();
  marker.ns = "artifacts";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  for (size_t i = 0; i < gt_artifacts.size(); i++) {
    tf::Pose tmp_pose = gt_artifacts[i].second;
    tf::poseTFToMsg(tmp_pose, marker.pose);

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.pose.position.z += 1.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.scale.z = 0.4;
    marker.text = gt_artifacts[i].first;
    markerarray.markers.push_back(marker);
    marker.id++;
    
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = true;
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.text = "";
    tmp_pose = gt_artifacts[i].second;
    tf::poseTFToMsg(tmp_pose, marker.pose);
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    markerarray.markers.push_back(marker);
    marker.id++;
  }
  prop_marker_pub.publish(markerarray);

  static visualization_msgs::MarkerArray delete_markers;
  // Draw all ground truth points in blue
  // Draw all failed artifact in red
  // Draw all good artifacts in green
  visualization_msgs::MarkerArray send_markers;

  prop_marker_pub.publish(delete_markers);
  delete_markers.markers.clear();
  marker.header.frame_id = "darpa";
  marker.header.stamp = current_time;
  marker.ns = "Subt_markers";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.lifetime = ros::Duration(0);
  marker.frame_locked = true;
  marker.color.r = 0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  for (size_t i = 0; i < good_artifacts.size(); i++) {
    marker.pose.position.x = std::get<1>(good_artifacts[i]).x();
    marker.pose.position.y = std::get<1>(good_artifacts[i]).y();
    marker.pose.position.z = std::get<1>(good_artifacts[i]).z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker text_marker = marker;
    text_marker.pose.position.z += 1.0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::get<0>(good_artifacts[i]);
    // send_markers.markers.push_back(text_marker);  // This is the name again
    marker.id++;
    visualization_msgs::Marker line_marker = marker;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.points.push_back(marker.pose.position);
    geometry_msgs::Point distal_pt;
    tf::pointTFToMsg(gt_artifacts[std::get<2>(good_artifacts[i])].second.getOrigin(), distal_pt);
    line_marker.points.push_back(distal_pt);
    makeMenuMarker(marker.id, marker.pose.position);
    inlier_interactive_index[std::get<4>(good_artifacts[i])] = marker.id;
    interactive_markers::MenuHandler::CheckState state;
    menu_handler[marker.id].getCheckState(h_mode_last[marker.id], state);
    if (state == interactive_markers::MenuHandler::CHECKED) {
      line_marker.color.a = 1.0;
    } else {
      line_marker.color.a = 0.3;
    }
    send_markers.markers.push_back(line_marker);
    line_marker.color.a = 1.0;
    line_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(line_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;

    // delete_markers.markers.push_back(text_marker);
    marker.id++;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::get<3>(good_artifacts[i]);
    text_marker.pose.position.x = (distal_pt.x + marker.pose.position.x)/2.0;
    text_marker.pose.position.y = (distal_pt.y + marker.pose.position.y)/2.0;
    text_marker.pose.position.z = (distal_pt.z + marker.pose.position.z)/2.0 - 0.5;
    text_marker.id = marker.id++;
    send_markers.markers.push_back(text_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(text_marker);
  }
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  for (size_t i = 0; i < bad_artifacts.size(); i++) {
    marker.pose.position.x = std::get<1>(bad_artifacts[i]).x();
    marker.pose.position.y = std::get<1>(bad_artifacts[i]).y();
    marker.pose.position.z = std::get<1>(bad_artifacts[i]).z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker text_marker = marker;
    text_marker.pose.position.z += 1.0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::get<0>(bad_artifacts[i]);
    // send_markers.markers.push_back(text_marker);
    marker.id++;
    visualization_msgs::Marker line_marker = marker;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.points.push_back(marker.pose.position);
    geometry_msgs::Point distal_pt;
    tf::pointTFToMsg(gt_artifacts[std::get<2>(bad_artifacts[i])].second.getOrigin(), distal_pt);
    line_marker.points.push_back(distal_pt);
    makeMenuMarker(marker.id, marker.pose.position);
    inlier_interactive_index[std::get<4>(bad_artifacts[i])] = marker.id;
    interactive_markers::MenuHandler::CheckState state;
    menu_handler[marker.id].getCheckState(h_mode_last[marker.id], state);
    if (state == interactive_markers::MenuHandler::CHECKED) {
      line_marker.color.a = 1.0;
    } else {
      line_marker.color.a = 0.3;
    }
    send_markers.markers.push_back(line_marker);
    line_marker.color.a = 1.0;
    line_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(line_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    // delete_markers.markers.push_back(text_marker);
    marker.id++;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::get<3>(bad_artifacts[i]);
    text_marker.pose.position.x = (distal_pt.x + marker.pose.position.x)/2.0;
    text_marker.pose.position.y = (distal_pt.y + marker.pose.position.y)/2.0;
    text_marker.pose.position.z = (distal_pt.z + marker.pose.position.z)/2.0 - 0.5;
    text_marker.id = marker.id++;
    send_markers.markers.push_back(text_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(text_marker);
  }
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  for (size_t i = 0; i < relative_frame_aligned_artifacts.size(); i++) {
    marker.pose.position.x = std::get<0>(relative_frame_aligned_artifacts[i]).x();
    marker.pose.position.y = std::get<0>(relative_frame_aligned_artifacts[i]).y();
    marker.pose.position.z = std::get<0>(relative_frame_aligned_artifacts[i]).z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker line_marker = marker;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.points.push_back(marker.pose.position);
    geometry_msgs::Point distal_pt;
    tf::pointTFToMsg(gt_artifacts[std::get<1>(relative_frame_aligned_artifacts[i])].second.getOrigin(), distal_pt);
    line_marker.points.push_back(distal_pt);
    send_markers.markers.push_back(line_marker);
    line_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(line_marker);
    marker.id++;
  }
  prop_marker_pub.publish(send_markers);

}
//time timestamp 					# time since sim bootup
//string reported_artifact_type			# artifact type from the report
//geometry_msgs/Point reported_artifact_position	# artifact position from the report
//string closest_artifact_name			# model name of the nearest artifact, e.g., backpack_4
//float64 distance				# distance to nearest artifact [m]
//int32 points_scored				# points scored in this attempt
//int32 total_score				# resulting total score after scoring this attempt

void LiveMapAnalyzer::ArtifactCallback(const map_analysis::ArtifactReport::ConstPtr& msg) {
  // Artifact report
  //
  tf::Point report_pt;
  tf::pointMsgToTF(msg->reported_artifact_position, report_pt);
  tf::Stamped<tf::Point> pt_stamped(report_pt, current_time, "darpa");
  relative_reports.push_back(std::make_pair(msg->reported_artifact_type, pt_stamped));
  //DoRelativeAssignment();
  // Go through all of the ground truth artifacts with the same type and find the closest
  std::vector<std::pair<std::string, tf::Pose>>::const_iterator closest_itr =
    gt_artifacts.end();
  double best_dist = std::numeric_limits<double>::infinity();
  double best_distxy = std::numeric_limits<double>::infinity();
  int ind = 0;
  int best_ind = -1;
  for (std::vector<std::pair<std::string, tf::Pose>>::const_iterator itr =
       gt_artifacts.begin();
       itr != gt_artifacts.end(); itr++, ind++) {
    if (itr->first == msg->reported_artifact_type) {
      double dist = report_pt.distance(itr->second.getOrigin());
      tf::Point pt_darpaxy(report_pt.x(), report_pt.y(), 0.0);
      tf::Point gt_pt_xy(itr->second.getOrigin().x(), itr->second.getOrigin().y(), 0.0);
      double distxy = pt_darpaxy.distance(gt_pt_xy);
      ROS_INFO_STREAM("Received artifact report with nearest type at dist " << dist);
      if (dist < best_dist) {
        best_dist = dist;
        closest_itr = itr;
        best_ind = ind;
      }
      if (distxy < best_distxy) {
        best_distxy = distxy;
      }
    }
  }
  if (closest_itr != gt_artifacts.end()) {
    // Classify good/bad artifact reports based on a threshold for display in RViz
    std::stringstream ss;
    ss << std::setprecision(2) << best_dist << "m";
    if (best_dist <= artifact_threshold) {
      ROS_INFO_STREAM(" A point scored with residual error of " << best_dist);
      points++;
      good_artifacts.push_back(std::make_tuple(msg->reported_artifact_type, report_pt, best_ind, ss.str(), relative_reports.size()-1));
    } else {
      bad_artifacts.push_back(std::make_tuple(msg->reported_artifact_type, report_pt, best_ind, ss.str(), relative_reports.size()-1));
    }
    if (best_distxy <= artifact_threshold) {
      pointsxy++;
    }
    residual = best_dist;
    residualxy = best_distxy;
    residuals.insert(residual);
    residualsxy.insert(residualxy);
    RMSE = getRMSE(residuals);
    RMSExy = getRMSE(residualsxy);
    if (residual < min_error)
      min_error = residual;
    if (residual > max_error)
      max_error = residual;
    if (residualxy < min_errorxy)
      min_errorxy = residualxy;
    if (residualxy > max_errorxy)
      max_errorxy = residualxy;
    // Fill out ArtifactMetrics msg
    map_analysis::ArtifactMetrics am;
    am.header.stamp = msg->timestamp;
    am.header.frame_id = "darpa";
    am.RMSE = RMSE;
    am.RMSE_xy = RMSExy;
    artifact_metrics_pub.publish(am);
    bag_out.write(artifact_metrics_pub.getTopic(), current_time, am);
  }
}

void LiveMapAnalyzer::RunStatusCallback(const map_analysis::RunStatus::ConstPtr& msg) {
  //  string status		# name of the current phase (setup, run, finished)
  //  time timestamp		# time since sim bootup
  // Logic here: We will be in free-running mode on startup to facilitate testing and offline operation
  // When we receive a setup or run message, then we zero out and reset all mapping metrics, dumping
  // all accumulated clouds. No new updates will be taken after the finished signal is received
  if (msg->status == "setup") {
    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator itr =  mr_clouds.begin();
         itr != mr_clouds.end(); itr++) {
      itr->second.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    relative_reports.clear();
    relative_reports.resize(0);
    good_artifacts.clear();
    good_artifacts.resize(0);
    bad_artifacts.clear();
    bad_artifacts.resize(0);
    master_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO_STREAM("Setting run status to SETUP");
  } else if (msg->status == "run") {
    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator itr =  mr_clouds.begin();
         itr != mr_clouds.end(); itr++) {
      itr->second.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    relative_reports.clear();
    relative_reports.resize(0);
    good_artifacts.clear();
    good_artifacts.resize(0);
    bad_artifacts.clear();
    bad_artifacts.resize(0);
    master_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    enabled = true;
    in_run = true;
    start_time = msg->timestamp.toSec();
    ROS_INFO_STREAM("Setting run status to RUN");
  } else if (msg->status == "finished") {
    enabled = false;
    ROS_INFO_STREAM("Setting run status to FINISHED");
  }
}

void LiveMapAnalyzer::RunStringCallback(const std_msgs::String::ConstPtr& msg) {
  //  string status		# name of the current phase (setup, run, finished)
  //  time timestamp		# time since sim bootup
  // Logic here: We will be in free-running mode on startup to facilitate testing and offline operation
  // When we receive a setup or run message, then we zero out and reset all mapping metrics, dumping
  // all accumulated clouds. No new updates will be taken after the finished signal is received
  if (msg->data == "init") {
    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator itr =  mr_clouds.begin();
         itr != mr_clouds.end(); itr++) {
      itr->second.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    relative_reports.clear();
    relative_reports.resize(0);
    good_artifacts.clear();
    good_artifacts.resize(0);
    bad_artifacts.clear();
    bad_artifacts.resize(0);
    master_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    //ROS_INFO_STREAM("Setting run status to SETUP");
  } else if (msg->data == "started" && in_run == false) {
    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator itr =  mr_clouds.begin();
         itr != mr_clouds.end(); itr++) {
      itr->second.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    relative_reports.clear();
    relative_reports.resize(0);
    good_artifacts.clear();
    good_artifacts.resize(0);
    bad_artifacts.clear();
    bad_artifacts.resize(0);
    master_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    enabled = true;
    in_run = true;
    //ROS_INFO_STREAM("Setting run status to RUN");
  } else if (msg->data == "finished") {
    enabled = false;
    //ROS_INFO_STREAM("Setting run status to FINISHED");
  }
}

void LiveMapAnalyzer::DoRelativeAssignment() {

  std::vector<int> best_relative_da;
  Eigen::MatrixXf relative_transform;
  std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> local_relative_reports;
  for (size_t i = 0; i < relative_reports.size(); i++) {
    if (inlier_interactive_index.find(i) != inlier_interactive_index.end()) {
      interactive_markers::MenuHandler::CheckState state;
      menu_handler[inlier_interactive_index[i]].getCheckState(h_mode_last[inlier_interactive_index[i]], state);
      if (state == interactive_markers::MenuHandler::UNCHECKED) {
        continue;
      }
    }
    local_relative_reports.push_back(relative_reports[i]);
  }
  relative_RMSE = GetBestRelativeDA(gt_artifacts, local_relative_reports, &best_relative_da, &relative_transform);
  std::cout << "Relative RMSE of " << relative_RMSE << " was achieved with assignment: " << std::endl;
  for (int i = 0; i < best_relative_da.size(); i++) {
    std::cout << i << " -> " << best_relative_da[i] << std::endl;
  }
  relative_frame_aligned_artifacts.clear();
  for (unsigned int i = 0; i < best_relative_da.size(); i++) {
    Eigen::MatrixXf tmp(4, 1);
    tmp(0, 0) = local_relative_reports[i].second.x();
    tmp(1, 0) = local_relative_reports[i].second.y();
    tmp(2, 0) = local_relative_reports[i].second.z();
    tmp(3, 0) = 1.0;
    Eigen::MatrixXf tmp2 = relative_transform * tmp;
    relative_frame_aligned_artifacts.push_back(std::make_tuple(tf::Point(tmp2(0, 0),
                                                                         tmp2(1, 0),
                                                                         tmp2(2, 0)),
                                                               best_relative_da[i]));
  }
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LiveMapAnalyzer::UnionClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_old,
                                                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_new) {
  // Might want to detect large updates that might represent overwrite map data -- then clear out cloud1
  // if cloud_new.size ~= cloud_old.size and cloud_new.size > const... then return cloud2
  //
  if (cloud_old->points.size() == 0) return cloud_new;
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr changestm(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierstm(new pcl::PointCloud<pcl::PointXYZ>);
  double ttom = getInliersInCloud(cloud_old, cloud_new, changestm, inlierstm, config_.incremental_inlier_tolerance);
  *result += *cloud_old;
  *result += *changestm;

  ROS_INFO_STREAM("In accumulate mode: Prior cloud of size " << cloud_old->points.size() <<
                  " and added a new cloud of size " << cloud_new->points.size() << " The overlap was " <<
                  inlierstm->points.size() << " the new part is of size " << changestm->points.size() <<
                  " and the resulting union is of size " << result->points.size());
  return result;
}

void LiveMapAnalyzer::MRCloudCallback(const std::string& robotname, const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Clouds from multirobot systems will be handled by keeping the most recent of each type, then merging them
  // and calling CloudCallback with the full set
  //
  // Convert to pcl::PointCloud
  // Ignore zero sized clouds
  // also do not take in new clouds when disabled
  if (!enabled || msg->height*msg->width == 0) return;
  ros::Time frame_time = current_time;
  frame_history.push_back(frame_time);
  while (frame_history.size() && (frame_history.size() > 20 && frame_history.back() - frame_history.front() > ros::Duration(20.0))) {
    frame_history.pop_front();
  }

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  if (mr_clouds.find(robotname) == mr_clouds.end())
    mr_clouds[robotname].reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (config_.accumulate_mode) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *(tmp));
    if (crop_starting_area)
    {
      cropper.setInputCloud(tmp);
      cropper.filter(*tmp);
    }
    mr_clouds[robotname] = UnionClouds(mr_clouds[robotname], tmp);
  } else {
    pcl::fromPCLPointCloud2(pcl_pc2, *(mr_clouds[robotname]));
    if (crop_starting_area)
    {
      cropper.setInputCloud(mr_clouds[robotname]);
      cropper.filter(*mr_clouds[robotname]);
    }
  }
}

void LiveMapAnalyzer::MRCloudRateCallback(const std::string& robotname, const std_msgs::Int32::ConstPtr& msg) {
  if (!enabled || msg->data == 0) return;
  total_cloud_size_ += msg->data - mr_cloud_sizes[robotname];
  mr_cloud_sizes[robotname] = msg->data;
}

void LiveMapAnalyzer::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // Convert to pcl::PointCloud
  // Ignore zero sized clouds
  // Also do not take in new cloud data when disabled
  if (!enabled || msg->height*msg->width == 0) return;
  ros::Time frame_time = current_time;
  frame_history.push_back(frame_time);
  while (frame_history.size() && (frame_history.size() > 20 && frame_history.back() - frame_history.front() > ros::Duration(20.0))) {
    frame_history.pop_front();
  }

  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*msg, *pcl_pc2);
  if (!master_cloud) master_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (config_.accumulate_mode) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_pc2, *(tmp));
    if (crop_starting_area)
    {
      cropper.setInputCloud(tmp);
      cropper.filter(*tmp);
    }
    pcl::toPCLPointCloud2(*tmp, *pcl_pc2);
    total_pre_ds_size_ += pcl_pc2->height * pcl_pc2->width;
    voxel_filter_.setInputCloud(pcl_pc2);
    voxel_filter_.setLeafSize(config_.min_leaf_size, config_.min_leaf_size, config_.min_leaf_size);
    pcl::PCLPointCloud2::Ptr ds_pc2 (new pcl::PCLPointCloud2 ());
    voxel_filter_.filter(*ds_pc2);
    total_post_ds_size_ += ds_pc2->height * ds_pc2->width;
    pcl::fromPCLPointCloud2(*ds_pc2, *(tmp));
    master_cloud = UnionClouds(master_cloud, tmp);
    int new_master_cloud_size = master_cloud->height * master_cloud->width;
    std::cout << "pre: " << total_pre_ds_size_ << " post: " << total_post_ds_size_ << " master size: " << new_master_cloud_size << std::endl;
    acc_cloud_size_est_ = int(total_pre_ds_size_ / total_post_ds_size_ * float(new_master_cloud_size));
  } else {
    pcl::fromPCLPointCloud2(*pcl_pc2, *master_cloud);
    if (crop_starting_area)
    {
      cropper.setInputCloud(master_cloud);
      cropper.filter(*master_cloud);
    }
  }
}
bool LiveMapAnalyzer::AnalyzeServiceCb(map_analysis::AnalyzePointCloud::Request& req,
                                       map_analysis::AnalyzePointCloud::Response& rsp) {
  ROS_INFO_STREAM("Analyze service callback called");
  // This is primarily a test interface to send large clouds to see where we lose frame rate
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(req.cloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
  AnalyzeCloud(temp_cloud);
  rsp.inlier_rate = map_coverage;
  rsp.outlier_rate = map_outliers;
  return true;
}

void LiveMapAnalyzer::ProcessClouds(const ros::TimerEvent& evt) {
  // Merge all available clouds into one mega cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (master_cloud)
    *temp_cloud += *master_cloud;
  for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator itr = mr_clouds.begin();
       itr != mr_clouds.end(); itr++) {
    *temp_cloud += (*itr->second);
  }
  if (temp_cloud->points.size() != 0) {
    if (config_.accumulate_mode || mr_clouds.size() != 0) {
      PublishCloud(accumulated_cloud_pub, temp_cloud);
    }
    AnalyzeCloud(temp_cloud);
  }
}

void LiveMapAnalyzer::ProcessClouds() {
  // Merge all available clouds into one mega cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (master_cloud)
    *temp_cloud += *master_cloud;
  for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator itr = mr_clouds.begin();
       itr != mr_clouds.end(); itr++) {
    *temp_cloud += (*itr->second);
  }
  if (temp_cloud->points.size() != 0) {
    if (config_.accumulate_mode || mr_clouds.size() != 0) {
      PublishCloud(accumulated_cloud_pub, temp_cloud);
    }
    AnalyzeCloud(temp_cloud);
  }
}

void LiveMapAnalyzer::AnalyzeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
  static ros::Time first_time = current_time;
  ros::Time frame_time = current_time;
  fps = frame_history.size() < 2 || frame_history.back() == frame_history.front() ?
    0.0 : static_cast<double>(frame_history.size()-1) / (frame_history.back() - frame_history.front()).toSec();
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud = input_cloud;
    
  // Downsample if necessary
  pcl::PCLPointCloud2::Ptr ds_pc2 (new pcl::PCLPointCloud2 ());
  ds_pc2->width = 0;
  ds_pc2->height = 0;
  pcl::PCLPointCloud2::Ptr in_pc2 (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*test_cloud, *in_pc2);
  voxel_filter_.setInputCloud(in_pc2);
  voxel_filter_.setLeafSize(config_.min_leaf_size, config_.min_leaf_size, config_.min_leaf_size);
  voxel_filter_.filter(*ds_pc2);

  if(ds_pc2->width != 0 && ds_pc2->height != 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*ds_pc2, *cloud_filtered);
    test_cloud = cloud_filtered;
  }
  
  if(total_cloud_size_ == 0)
    cloud_size = test_cloud->points.size();
  else
    cloud_size = total_cloud_size_;
  if(config_.accumulate_mode && acc_cloud_size_est_ > 0)
    cloud_size = acc_cloud_size_est_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr changestm(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr changesmt(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierstm(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliersmt(new pcl::PointCloud<pcl::PointXYZ>);
  std::pair<double, double> errs =
    ComputeChanges(test_cloud, model_, changestm, inlierstm, changesmt, inliersmt,
                   config_.inlier_tolerance, config_.outlier_tolerance);
  //  double coverage = 1.0 - static_cast<double>(changestm->size()) / static_cast<double>(model->size());
  map_coverage = static_cast<double>(inlierstm->size()) / static_cast<double>(model_->size());
  map_outliers = static_cast<double>(changesmt->size()) / static_cast<double>(test_cloud->size());
  PublishPropMarkers();
  PublishCloud(outlier_cloud_pub, changesmt);
  //PublishCloud(inlier_cloud_pub, inlierstm); // visualize ground truth coverage
  PublishCloud(inlier_cloud_pub, inliersmt); // visualize submitted inliers
  if(!pub_once) {
    PublishCloud(full_cloud_pub, model_);
    pub_once = true;
  }

  if(enable_mesh_analysis_) {
    std::tuple<double, double> mesh_results =
      gtMesh_.DistCheck(*test_cloud);
    mesh_face_coverage = std::get<1>(mesh_results);
    mesh_density = std::get<0>(mesh_results);
    // We want to track the density of the original pointclouds provided before downsampling
    mesh_density *= float(total_cloud_size_) / float(test_cloud->points.size());
    gtMesh_.PublishMesh();
  }
  PublishJskVisuals();
  ROS_INFO_STREAM("Processed cloud with " << floor(test_cloud->points.size()/1000) << "k points against a model cloud with "
                   << floor(model_->points.size()/1000) << "k points in " << (current_time-frame_time).toSec() << " seconds");
  // Fill out MapMetrics msg
  map_analysis::MapMetrics mm;
  mm.header.seq = msg_sequence;
  mm.header.stamp = frame_time;
  mm.header.frame_id = "darpa"; // todo: should be dynamic
  // uint8 MAP_MULTI=0 # separate maps per robot
  // uint8 MAP_PARTIAL=1 # maps covering a portion of covered area
  // uint8 MAP_MERGED=2 # single merged map covering full extent of covered area
  mm.map = 2; // type of submitted map, defined by constants - defaulting to 2 for now
  mm.coverage = map_coverage*100.0; // inliers compared to ground truth, %
  mm.error = map_outliers*100.0; // outliers compared to ground truth, %
  mm.size = cloud_size; // number of points
  mm.rate = fps; // updates per second, Hz
  mm.density = mesh_density; // avg points per surface patch for each patch covered - defauting to 0 for now
  map_metrics_pub.publish(mm);
  bag_out.write(map_metrics_pub.getTopic(), current_time, mm);
}
void LiveMapAnalyzer::PublishJskVisuals() {
  // uint8 ADD=0
  // uint8 DELETE=1
  // uint8 action
  // int32 width
  // int32 height
  // int32 left
  // int32 top
  // std_msgs/ColorRGBA bg_color
  //   float32 r
  //   float32 g
  //   float32 b
  //   float32 a
  // int32 line_width
  // float32 text_size
  // string font
  // std_msgs/ColorRGBA fg_color
  //   float32 r
  //   float32 g
  //   float32 b
  //   float32 a
  // string text
  std_msgs::ColorRGBA SubT_medium_blue;
  SubT_medium_blue.r = 0.059;
  SubT_medium_blue.g = 0.369;
  SubT_medium_blue.b = 0.565;
  SubT_medium_blue.a = 1.0;
  std_msgs::ColorRGBA SubT_bright_red_orange;
  SubT_bright_red_orange.r = 1.0;
  SubT_bright_red_orange.g = 0.275;
  SubT_bright_red_orange.b = 0.0;
  SubT_bright_red_orange.a = 1.0;
  std_msgs::ColorRGBA SubT_orange;
  SubT_orange.r =  0.898;
  SubT_orange.g = 0.451;
  SubT_orange.b = 0.0;
  SubT_orange.a = 1.0;
  std_msgs::ColorRGBA SubT_green;
  SubT_green.r = 0.596;
  SubT_green.g = 0.722;
  SubT_green.b = 0.416;
  SubT_green.a = 1.0;
  std_msgs::ColorRGBA SubT_light_blue;
  SubT_light_blue.r = 0.698;
  SubT_light_blue.g = 0.741;
  SubT_light_blue.b = 0.773;
  SubT_light_blue.a = 1.0;
  std_msgs::ColorRGBA SubT_medium_gray;
  SubT_medium_gray.r = 0.490;
  SubT_medium_gray.g = 0.431;
  SubT_medium_gray.b = 0.373;
  SubT_medium_gray.a = 1.0;
  std_msgs::ColorRGBA SubT_white;
  SubT_white.r = 1.0;
  SubT_white.g = 1.0;
  SubT_white.b = 1.0;
  SubT_white.a = 1.0;
  std_msgs::ColorRGBA SubT_black;
  SubT_black.r = 0.0;
  SubT_black.g = 0.0;
  SubT_black.b = 0.0;
  SubT_black.a = 1.0;
}
void LiveMapAnalyzer::PublishCloud(ros::Publisher& pub,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  sensor_msgs::PointCloud2 msg;
  msg.data.resize(12 * cloud->points.size());
  msg.width = cloud->points.size();
  msg.height = 1;
  msg.row_step = cloud->points.size() * 12;
  msg.is_dense = true;
  sensor_msgs::PointField fx;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = 7;
  fx.count = 1;
  msg.fields.push_back(fx);
  fx.name = "y";
  fx.offset = 4;
  msg.fields.push_back(fx);
  fx.name = "z";
  fx.offset = 8;
  msg.fields.push_back(fx);
  msg.point_step = 12;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    *reinterpret_cast<float*>(&msg.data[i*12 + 0]) = cloud->points[i].x;
    *reinterpret_cast<float*>(&msg.data[i*12 + 4]) = cloud->points[i].y;
    *reinterpret_cast<float*>(&msg.data[i*12 + 8]) = cloud->points[i].z;
  }

  msg.header.frame_id = "darpa";
  msg.header.stamp = current_time;
  pub.publish(msg);
  msg.header.stamp = current_time;
  if(in_run)
    bag_out.write(pub.getTopic(), current_time, msg);
}

// void LiveMapAnalyzer::PublishCloud(ros::Publisher& pub,
//                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//  sensor_msgs::PointCloud msg;
////  pcl::toROSMsg(*cloud, msg);
//  msg.points.resize(cloud->points.size());
//  for (size_t i = 0; i < cloud->points.size(); i++) {
//    geometry_msgs::Point32 pt;
//    pt.x = cloud->points[i].x;
//    pt.y = cloud->points[i].y;
//    pt.z = cloud->points[i].z;
//    msg.points[i] = pt;
//  }
//  msg.header.frame_id = "darpa";
//  msg.header.stamp = current_time;
//  pub.publish(msg);
//}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_map_analysis");
  LiveMapAnalyzer analyzer;
  
  std::string bag_in_name, bag_out_name;
  int max_time;
  analyzer.private_nh.param("bag_in", bag_in_name, std::string("none"));
  analyzer.private_nh.param("bag_out", bag_out_name, std::string("none"));
  analyzer.private_nh.param("max_duration_sec", max_time, 10000);
  
  bool full_run, accumulate_mode;
  analyzer.private_nh.param("full_run", full_run, true);
  analyzer.private_nh.param("accumulate_mode", accumulate_mode, false);
  double inlier_tolerance, outlier_tolerance, min_leaf_size, incremental_inlier_tolerance;
  analyzer.private_nh.param("inlier_tolerance", inlier_tolerance, 1.0);
  analyzer.private_nh.param("outlier_tolerance", outlier_tolerance, 1.0);
  analyzer.private_nh.param("min_leaf_size", min_leaf_size, 0.05);
  analyzer.private_nh.param("incremental_inlier_tolerance", incremental_inlier_tolerance, 0.1);
  
  rosbag::Bag bag_in;
  ROS_INFO("Processing bag %s to save new bag %s", bag_in_name.c_str(), bag_out_name.c_str());
  bag_in.open(bag_in_name.c_str());
  analyzer.bag_out.open(bag_out_name.c_str(), rosbag::bagmode::Write);
  analyzer.bag_out.setCompression(rosbag::compression::BZ2);
  
  rosbag::View view(bag_in);
  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
  std::vector<std::string> topics;

  BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
    // virtual+systems clouds
    std::size_t found = info->topic.rfind("cloud");
    if (found!=std::string::npos) {
      cout << info->topic << "\t[" << info->datatype << "]\t" << endl;
      std::string cloudname = info->topic;
      cloudname.erase(found,cloudname.size());
      cloudname.erase(0,1);
      
      std::size_t found_sub = info->topic.rfind("/cloud_initial_size");
      if (found_sub!=std::string::npos) {
        // cloud initial sizes
        analyzer.mr_cloud_sizes[cloudname] = 0;
      }
      // add cloud names to lookup
      topics.push_back(info->topic);
    }
    
    // virtual+systems markers
    found = info->topic.rfind("/marker");
    if (found!=std::string::npos) {
      cout << info->topic << "\t[" << info->datatype << "]\t" << endl;
      // add markers to lookup
      topics.push_back(info->topic);
    }
    
    // systems events/other
    found = info->topic.rfind("subt/artifact_reports");
    if (found!=std::string::npos) {
      cout << info->topic << "\t[" << info->datatype << "]\t" << endl;
      // add markers to lookup
      topics.push_back(info->topic);
    }
    found = info->topic.rfind("/tf");
    if (found!=std::string::npos) {
      cout << info->topic << "\t[" << info->datatype << "]\t" << endl;
      // add markers to lookup
      topics.push_back(info->topic);
    }
  }
  // add subt/start for run status change
  topics.push_back("/subt/start");
  
  // init config
  analyzer.ConfigInit(accumulate_mode,inlier_tolerance,outlier_tolerance,min_leaf_size,incremental_inlier_tolerance);

  // lookup these specfic topics from the bag
  rosbag::View cloud_view(bag_in, rosbag::TopicQuery(topics));
  ros::Time next_analysis_time = ros::Time(0);
  bool current = false;
  foreach(rosbag::MessageInstance const m, view)
  {
    analyzer.current_time = m.getTime();
  
    std::string cloudname = m.getTopic();
    std::size_t found = cloudname.rfind("cloud");
    if (found!=std::string::npos) {
      std::size_t found_sub = cloudname.rfind("/cloud_initial_size");
      if (found_sub!=std::string::npos) {
        // Handle cloud sizes
        cloudname.erase(found,cloudname.size());
        cloudname.erase(0,1);
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL) {
          analyzer.MRCloudRateCallback(cloudname, i);
        }
      } else
      {
        // Handle clouds
        cloudname.erase(found,cloudname.size());
        cloudname.erase(0,1);
        sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc != NULL) {
          analyzer.MRCloudCallback(cloudname, pc);
          current = false;
        }
      }
    } else {
      found = m.getTopic().rfind("/subt/start");
      if (found!=std::string::npos) {
        // Handle run status messages
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL) {
          analyzer.RunStringCallback(s);
          if(analyzer.InRun()) {
            analyzer.bag_out.write(m.getTopic(), m.getTime(), s);
          }
        }
      } else {
        found = m.getTopic().rfind("/markers");
        if (found!=std::string::npos) {
          // Handle run status messages
          visualization_msgs::MarkerArray::ConstPtr ma = m.instantiate<visualization_msgs::MarkerArray>();
          if (ma != NULL) {
            if(analyzer.InRun())
              analyzer.bag_out.write(m.getTopic(), m.getTime(), ma);
          }
        } else {
          found = m.getTopic().rfind("/marker");
          if (found!=std::string::npos) {
            // Handle run status messages
            visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
            if (marker != NULL) {
              if(analyzer.InRun())
                analyzer.bag_out.write(m.getTopic(), m.getTime(), marker);
            }
          } else {
            found = m.getTopic().rfind("/tf");
            if (found!=std::string::npos) {
              // Handle run status messages
              tf2_msgs::TFMessage::ConstPtr tfm = m.instantiate<tf2_msgs::TFMessage>();
              if (tfm != NULL) {
                if(analyzer.InRun())
                  analyzer.bag_out.write(m.getTopic(), m.getTime(), tfm);
              }
            } else {
              found = m.getTopic().rfind("subt/artifact_reports");
              if (found!=std::string::npos) {
                // Handle run status messages
                map_analysis::ArtifactReport::ConstPtr ar = m.instantiate<map_analysis::ArtifactReport>();
                if (ar != NULL) {
                  if(analyzer.InRun())
                    analyzer.bag_out.write(m.getTopic(), m.getTime(), ar);
                }
              } else {
                found = m.getTopic().rfind("/subt/status");
                if (found!=std::string::npos) {
                  // Handle run status messages
                  map_analysis::RunStatus::ConstPtr rs = m.instantiate<map_analysis::RunStatus>();
                  if (rs != NULL) {
                    analyzer.RunStatusCallback(rs);
                    if(analyzer.InRun()) {
                      analyzer.bag_out.write(m.getTopic(), m.getTime(), rs);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    if(analyzer.InRun() && m.getTime() > next_analysis_time && current == false && full_run) {
      if(analyzer.mr_clouds.size() > 0)
        analyzer.ProcessClouds();
      next_analysis_time = next_analysis_time + ros::Duration(10);
      current = true;
    }
    double elapsed_time = m.getTime().toSec() - analyzer.start_time;
    if(analyzer.InRun() && elapsed_time >= max_time) {
      if(analyzer.mr_clouds.size() > 0)
        analyzer.ProcessClouds();
      break;
    }
  }
  if(full_run == false)
    if(analyzer.mr_clouds.size() > 0)
        analyzer.ProcessClouds();
  bag_in.close();
  analyzer.bag_out.close();
  ROS_INFO_STREAM("Mapping bag analysis complete. Saved to new rosbag.");

  return 0;
}

