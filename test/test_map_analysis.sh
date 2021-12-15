#/bin/bash
# Local Config
. $PWD/../../../devel/setup.bash
TEST_DIR=$PWD

echo Running on $ROS_MASTER_URI

roslaunch map_analysis map_analysis.launch env_pcd:=$(rospack find map_analysis)/test/SubTLogo.pcd env_gt:=$(rospack find map_analysis)/test/artifacts.csv &
MAP_ANALYZER=$!
python $TEST_DIR/test_pcl2.py /cloud &
PCL_PID1=$!
python $TEST_DIR/test_pcl2.py /clouds/robot1
PCL_PID2=$!

kill $MAP_ANALYZER $PCL_PID1 $PCL_PID2

