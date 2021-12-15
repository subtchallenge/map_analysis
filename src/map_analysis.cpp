#include <map_analysis/analysis.h>

int main(int argc, char** argv) {

  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("noviz", "Don't show the visualization")
    ("model", boost::program_options::value<std::string>(), "model")
    ("outfile", boost::program_options::value<std::string>(), "output file")
    ("outcloud", boost::program_options::value<std::string>(), "output cloud")
    ("animate_frames", boost::program_options::value<size_t>(), "Skip this many frames on animation of output cloud. Default = 0 -> only draw final ish cloud. 1 means no skip")
    ("start_time", boost::program_options::value<std::string>(), "Start time of run. If specified, must also specify end time. Without this parameter, will use entire bag")
    ("end_time", boost::program_options::value<std::string>(), "End time of run. If specified, must also specify start time. Without this parameter, will use entire bag data")
    ("input-files", boost::program_options::value<std::vector<std::string> >(), "input bag file(s?)");
  boost::program_options::positional_options_description pod;
  pod.add("input-files", -1);

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pod).run(), vm);
  boost::program_options::notify(vm);
  if (vm.count("help")) {
    std::cout << desc <<std::endl;
    return 1;
  }
  ofstream outfile;
  std::string outpcdfilename;
  if (vm.count("outfile") == 0) {
    std::cerr << " Please specify an output file to save coverage statistics if desired " << std::endl;
  } else {
    outfile.open(vm["outfile"].as<std::string>(), std::ios::out);
    std::stringstream ss;
    ss << vm["outfile"].as<std::string>() << ".pcd";
    outpcdfilename = ss.str();
  }
  if (vm.count("outcloud")) {
    if (vm.count("animate_frames") == 0 || vm["animate_frames"].as<size_t>() == 0) {
      outpcdfilename = vm["outcloud"].as<std::string>();
    } else {
      boost::filesystem::path temp(vm["outcloud"].as<std::string>());
      if (temp.has_extension()) {
        outpcdfilename = temp.stem().string();
      } else {
        outpcdfilename = temp.string();
      }
    }
    std::cout << "Saving output cloud pattern to " << outpcdfilename << std::endl;
  }

  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (!vm.count("model")) {
    std::cout << "Need to specify model file!" << std::endl;
    return 1;
  }
  std::string modelfname = vm["model"].as<std::string>();
  auto model = pcl::PointCloud<POINT_T>::Ptr(new pcl::PointCloud<POINT_T>);
  PCL_INFO("Loading a pcd file %s\n", modelfname.c_str());
  if (pcl::io::loadPCDFile<POINT_T> (modelfname, *model) == -1) {
    PCL_ERROR("Couldn't read file %s\n", modelfname.c_str());
    return 1;
  }
  std::cout << "Loaded "
    << model->width * model->height
    << " data points from" << modelfname
    << std::endl;
  double start_time, end_time;
  if (vm.count("start_time") && vm.count("end_time")) {
    start_time = datestr_to_epoch(vm["start_time"].as<std::string>());
    end_time = datestr_to_epoch(vm["end_time"].as<std::string>());
  } else if (vm.count("start_time") || vm.count("end_time")) {
    std::cerr << "Need both start and end time if either is specified" << std::endl;
    return 1;
  }
  ros::Time first_time;
  if (vm.count("input-files")) {
    std::vector<std::pair<pcl::PointCloud<POINT_T>::Ptr, std::string> > clouds;
    for (auto itr = vm["input-files"].as<std::vector<std::string> >().begin();
         itr != vm["input-files"].as<std::vector<std::string> >().end();
         itr++) {
      rosbag::Bag bag;
      bag.open(*itr, rosbag::bagmode::Read);

      std::vector<std::string> topics;
      topics.push_back(std::string("/cloud"));

      rosbag::View view(bag, rosbag::TopicQuery(topics));
      ros::Time run_end_time;
      ros::Time run_start_time;
      if (vm.count("start_time") == 0) {
        foreach(rosbag::MessageInstance const m, view) {
          run_end_time = m.getTime();
        }  // We are assuming that the last cloud that we get is the end of the run, so we should start
        run_start_time = run_end_time - ros::Duration(60 * 60);
      } else {
        run_start_time = ros::Time(start_time);
        run_end_time = ros::Time(end_time);
      }

      size_t num_valid_msgs = 0;
      foreach(rosbag::MessageInstance const m, view) {
        if (m.getTime() < run_start_time || m.getTime() > run_end_time) {
          continue;
        } else {
          num_valid_msgs++;
        }
      }  // We are assuming that the last cloud that we get is the end of the run, so we should start
      std::cout << std::fixed << "Start time : " << run_start_time.toSec() << " End time : " << run_end_time.toSec()
        << " Have " << num_valid_msgs << " messages in this interval " << std::endl;

      bool save_once = false;
      size_t cloud_ind = 0;
      size_t anim_ind = 0;
      foreach(rosbag::MessageInstance const m, view) {
        if (m.getTime() < run_start_time || m.getTime() > run_end_time) {
          //    std::cout << "Skipping map data sent during setup time at : " << m.getTime() << " < "
          //     << run_start_time << std::endl;
          continue;
        }
        sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
        if (s != NULL) {
          // Convert to pcl::PointCloud
          if (s->height*s->width == 0) continue;
          if (first_time == ros::Time()) {
            first_time = run_start_time;  // First cloud came in sometime after the start..
            std::cout << " Skipped setup time of " << (m.getTime() - run_start_time).toSec() << " seconds" << std::endl;
          }
          if (vm.count("outcloud") && vm.count("animate_frames") && vm["animate_frames"].as<size_t>() > 0) {
            if (cloud_ind % vm["animate_frames"].as<size_t>() != 0) {
              if (!outfile.is_open()) {
                cloud_ind++;
                continue;  // Nobody needs this frame, skip ahead
              }
            }
          }
          pcl::PCLPointCloud2 pcl_pc2;
          pcl_conversions::toPCL(*s, pcl_pc2);
          pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
          pcl::PointCloud<pcl::PointXYZ>::Ptr changestm(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointCloud<pcl::PointXYZ>::Ptr changesmt(new pcl::PointCloud<pcl::PointXYZ>);
          std::pair<double, double> errs =
            ComputeChanges(temp_cloud, model, changestm, boost::none, changesmt, boost::none);
          double coverage = 1.0 - static_cast<double>(changestm->size()) / static_cast<double>(model->size());
          double error = static_cast<double>(changesmt->size()) / static_cast<double>(temp_cloud->size());

          if (outfile.is_open()) {
            outfile << (m.getTime() - first_time).toSec() << ", " << coverage << ", " << error << std::endl;
            // outfile << (m.getTime() - first_time).toSec() << ", " << errs.first << ", " << errs.second << std::endl;
            outfile.flush();
          }
          std::stringstream ss;
          ss << "MapAnalysisView: coverage: " << std::setprecision(2) << coverage*100.0 << "%   error: " << error *100.0 << "%";

          if (vm.count("outcloud") && !save_once && run_end_time - m.getTime() < ros::Duration(10) && (vm.count("animate_frames") == 0 || vm["animate_frames"].as<size_t>() == 0)) {
            save_cloud(outpcdfilename, temp_cloud, changestm, changesmt);
            save_once = true;
          } else {
            if (vm.count("outcloud") && vm.count("animate_frames") && vm["animate_frames"].as<size_t>() > 0) {
              if (cloud_ind % vm["animate_frames"].as<size_t>() == 0) {
                std::stringstream ss;
                ss << outpcdfilename << std::setw(4) << std::setfill('0') << anim_ind++ << ".pcd";
                save_cloud(ss.str(), temp_cloud, changestm, changesmt);
                std::cout << "Saved cloud to " << ss.str() << std::endl;
              }
            }
          }
          if (!vm.count("noviz")) {
            map_analysis_viewer(viewer, ss.str(), temp_cloud, changestm, changesmt, model);
            viewer->setWindowName(ss.str());
            viewer->spinOnce(100);
          }
          std::cout << std::fixed << std::setprecision(1) << (m.getTime() - first_time).toSec() << " : "
            << static_cast<double>(cloud_ind) / static_cast<double>(num_valid_msgs) * 100.0 << "%         \r";
          std::cout.flush();
          std::cout << std::scientific;
          cloud_ind++;
        }
      }
      std::cout << std::endl;
      bag.close();
    }
  }
  if (outfile.is_open())
    outfile.close();
}

