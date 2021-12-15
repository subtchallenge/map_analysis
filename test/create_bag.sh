#/bin/bash
# Local Config
. $PWD/../../../devel/setup.bash
TEST_DIR=$PWD

echo Running on $ROS_MASTER_URI

roslaunch map_analysis map_analysis.launch env_pcd:=$(rospack find map_analysis)/test/SubTLogo.pcd env_gt:=$(rospack find map_analysis)/test/artifacts.csv rviz:=true sim_time:=false &
MAP_ANALYZER=$!
python $TEST_DIR/test_pcl2.py /cloud &
PCL_PID1=$!
python $TEST_DIR/test_pcl2.py /clouds/robot1 &
PCL_PID2=$!
sleep 15; 
rostopic pub /subt/artifact_reports map_analysis/ArtifactReport "timestamp: {secs: 15, nsecs: 0}
reported_artifact_type: 'cube'
reported_artifact_position: {x: 0.0, y: 0.0, z: 0.5}
closest_artifact_name: 'cube'
distance: 0.5
points_scored: 1
total_score: 1" &
sleep 10;
rostopic pub /subt/artifact_reports map_analysis/ArtifactReport "timestamp: {secs: 25, nsecs: 0}
reported_artifact_type: 'gas'
reported_artifact_position: {x: 12.1, y: 0.0, z: 5.0}
closest_artifact_name: 'gas'
distance: 5.03
points_scored: 0
total_score: 1" &
sleep 10;
rostopic pub /subt/artifact_reports map_analysis/ArtifactReport "timestamp: {secs: 35, nsecs: 0}
reported_artifact_type: 'backpack'
reported_artifact_position: {x: 0.0, y: 1.0, z: 5.0}
closest_artifact_name: 'backpack'
distance: 1.0
points_scored: 1
total_score: 2" &
sleep 10;
rostopic pub /subt/artifact_reports map_analysis/ArtifactReport "timestamp: {secs: 45, nsecs: 0}
reported_artifact_type: 'survivor'
reported_artifact_position: {x: 7.07, y: 0.0, z: 0.1}
closest_artifact_name: 'survivor'
distance: 5.1
points_scored: 0
total_score: 2" &
sleep 10;
rostopic pub /subt/artifact_reports map_analysis/ArtifactReport "timestamp: {secs: 55, nsecs: 0}
reported_artifact_type: 'rope'
reported_artifact_position: {x: 0.0, y: 7.1, z: 5.0}
closest_artifact_name: 'rope'
distance: 0.1
points_scored: 1
total_score: 3"
kill $MAP_ANALYZER $PCL_PID1 $PCL_PID2

