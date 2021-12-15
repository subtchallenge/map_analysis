#/bin/bash
# Local Config
. $PWD/../../../devel/setup.bash
TEST_DIR=$PWD

echo Running on $ROS_MASTER_URI

roslaunch map_analysis bag_map_analysis.launch env_pcd:=$(rospack find map_analysis)/test/SubTLogo.pcd env_gt:=$(rospack find map_analysis)/test/artifacts.csv rviz:=false bag_in:=$(rospack find map_analysis)/test/input_map.bag bag_out:=$(rospack find map_analysis)/test/output_map.bag

