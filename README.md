# Map Analysis

This repository contains the Map Analysis package to evaluate the quality of mapping data using several intrinsic and comparison-based metrics. 
A version of this package was utilized to analyze competitor-submitted maps in real time during the DARPA Subterranean Challenge Final Event. The mapping data and metrics were presented throughout the [SubT Challenge Finals livestream](https://www.youtube.com/watch?v=EAPSm7udG3Q).

A publication on this work is in progress. For now, if you use this code in an academic context, please cite this repository.

## Description of Mapping Metrics

The Map Analysis package generates metrics for maps in the 3D point cloud format. Three of the metrics are intrinsic qualities of the map:

* Size -- total number of points in the point cloud map, which may be accumulated across multiple clouds when partial maps are provided
* Rate -- frequency of map updates (Hz)
* Type -- method of producing the map, especially when using multiple agents
  * Multi -- each map message may be provided by different agents
  * Partial -- each map message may provide only a portion of the total area mapped, i.e., submaps
  * Merged -- each map message is cumulative and includes the most complete information available

The remaining map metrics are calculated based on comparison of the input map against a known "ground truth" point cloud produced by a) scanning real-world environments with surveying equipment, and b) uniformly sampling a known mesh for virtual environments. 

* Inlier Cloud -- points in the input map within a (configurable) tolerance from the closest ground truth point
* Outlier Cloud -- points in the input map greater a (configurable) tolerance from the closest ground truth point
* Coverage -- percentage of ground truth points within a (configurable) distance of an input map point, i.e., `<ground truth points within distance of an inlier point> / <total ground truth points>`
* Deviation -- percentage of input map points greater than a (configurable) distance from the nearest ground truth point, i.e., `<outlier points> / <total submitted points>`

The inlier and outlier clouds are published on the ROS topics `MA_inlier_cloud` and `MA_outlier_cloud`, and the remaining numerical metrics are published on the `map_metrics` topic. 

While this package has only been tested using data from the SubT Challenge environments, this software is compatible with other environments provided a sufficiently accurate and dense ground truth pointcloud is known.

## Installation

This code has been tested using Ubuntu 18.04 and ROS Melodic. The following packages must also be installed:

```
sudo apt-get -y install ros-melodic-rosbag ros-melodic-dynamic-reconfigure libboost-program-options1.65.1 libpcl-common1.8 libpcl-octree1.8 libpcl-io1.8 libpcl-visualization1.8 libpcl-segmentation1.8
```

Then, set up and build this package in a catkin workspace.

```
mkdir -p mapping_analysis_ws/src
cd mapping_analysis_ws
catkin init

git clone git@github.com:subtchallenge/map_analysis.git src/map_analysis
catkin build
```

## Launch Real-Time Analysis 

An example launch file is provided for real-time map analysis and can be launched with:

```
cd mapping_analysis_ws
source devel/setup.bash
roslaunch map_analysis map_analysis.launch
```

By default, this will listen for input maps on the `/cloud` and `/clouds/*` topics (the latter for partial maps or maps submitted by multiple agents) and will compare the maps against a test point cloud represented as the SubT Challenge logo.
You will see the white ground truth cloud points in RViz.

Now, let's provide a test input map to analyze by running:

```
cd src/map_analysis/test
python test_pcl2.py /cloud
```

Now you will see new points in RViz -- the green inlier cloud and red outlier cloud. Every time a map is processed, you can also see the metrics with `rostopic echo /map_metrics`.

![Map Cube](https://github.com/subtchallenge/map_analysis/test/map_cube.gif)

## Changing Parameters

The launch file at `src/map_analysis/launch/map_analysis.launch` allows you to easily change the following parameters:

* `env_pcd`: The path to the ground truth point cloud file for a given environment (string, default: SubTLogo.pcd)  
* `rviz`: Whether or not to launch RViz (bool, default: false)  
* `env_gt`: The file containing the artifacts in a given environment (string, default: artifacts.csv)  
* `mesh_analysis`: Enable mesh analysis (bool, default: false)  

Additional parameters can be adjusted during runtime using Dynamic Reconfiguration (i.e. `rosrun rqt_reconfigure rqt_reconfigure`)  

* `inlier tolerance`: The max range (in meters) a point can be from a ground truth point and be considered an inlier (float, default: 1.0)  
* `outlier tolerance`: The range (in meters) at which a submitted point differs enough from ground truth to be considered an outlier (float, default: 1.0)  
* `accumulate_mode`: Whether or not to accumulate successive pointclouds, use this when sending differential updates as opposed to the entire map every message (bool, default: true)  
* `min_leaf_size`: The minimum leaf size to be applied for cloud downsampling prior to map analysis when necessary (float, default: 0.05)  
* `incremental_inlier_tolerance`: Tolerance to use in accumulated point cloud for union operation (float, default: 0.1)  

You can also remap from the default ROS topics within the launch file. For example, to listen for input maps on `/map_cloud`, you can add `<remap from="cloud" to="/map_cloud">` within the map analysis node tag.

### Offline ROS Bag Analysis 

The `map_analysis` package has ability to process bagged pointcloud data in an offline mode to allow for an adequate amount of runtime to generate map metrics as quickly as possible without the possibility of skipping map updates.  An example launch file is located under `src/map_analysis/launch/bag_map_analysis.launch` with the following additional parameters:

* `bag_in`: Offline bag input file (string, default: input_map.bag)  
* `bag_out`: Offline bag output file (string, default: output_map.bag)  

The resulting processed bag will contain analysis topics in a compressed format with all of the map metrics and segmented pointclouds.  
A quick method to view the resulting map metrics for the entire run from the bag is to run `rostopic echo -b <bag_out> /map_metrics`

The `map_analysis` package can also be run alongside an active bag playback to generate results with the `sim_time` parameter set to true.  
This method should use the live analysis mode, not the offline bag analysis mode.

### Ground Truth Point Clouds

When analyzing mapping data from the SubT Challenge events, please utilize the following repositories to download ground truth point cloud (`.pcd`) files which are compatible with the Map Analysis package:

* [Systems Competition - Tunnel Circuit](https://bitbucket.org/subtchallenge/tunnel_ground_truth)
* [Systems Competition - Urban Circuit](https://bitbucket.org/subtchallenge/urban_ground_truth)
* [Systems Competition - Final Event](https://bitbucket.org/subtchallenge/finals_ground_truth)
* [Virtual Competition - All Events](https://github.com/subtchallenge/virtual_ground_truth)

These can be cloned within your `mapping_analysis_ws` and loaded by using the path to a pcd file as your `env_pcd` argument during launch.
 
### Contact Information ###

If you have any questions or comments about this repository, please contact:

* Arthur Schang - arthur.c.schang2.ctr@army.mil  
* John Rogers - john.g.rogers59.civ@army.mil  
* Angela Maio - angela.c.maio.civ@army.mil  
* SubT Challenge Mailbox - SubTChallenge@darpa.mil  
