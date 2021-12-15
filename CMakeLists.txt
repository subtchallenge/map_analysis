cmake_minimum_required(VERSION 2.8.3)
project(map_analysis)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  interactive_markers
  message_generation
)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(PCL REQUIRED common octree io visualization segmentation)
find_package(assimp REQUIRED)
find_package( PkgConfig REQUIRED)
pkg_check_modules( flann REQUIRED flann )
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message ("OpenMP not found!!!")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ArtifactReport.msg
   ArtifactMetrics.msg
   MapMetrics.msg
   RunStatus.msg 
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   AnalyzePointCloud.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/MapAnalyzer.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES map_analysis
  CATKIN_DEPENDS rosbag sensor_msgs visualization_msgs dynamic_reconfigure message_runtime interactive_markers message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${assimp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${flann_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(analysis
  src/analysis.cpp
  src/relative_metric.cpp
)
target_link_libraries(analysis 
  ${PCL_LIBRARIES})
add_library(mesh
  src/mesh.cpp
)
target_link_libraries(mesh 
  ${PCL_LIBRARIES}
  ${flann_LIBRARIES}
  assimp)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/map_analysis.cpp)
add_executable(live_map_analysis src/live_map_analysis.cpp)
add_executable(test_map_analysis src/test_map_analysis.cpp)
add_executable(bag_map_analysis src/bag_map_analysis.cpp)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(live_map_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_map_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bag_map_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   mesh
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   analysis
 )
if(OpenMP_CXX_FOUND)
 target_link_libraries(live_map_analysis
   mesh
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   analysis
   OpenMP::OpenMP_CXX
 )
else() 
 target_link_libraries(live_map_analysis
   mesh
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   analysis
 )
endif()

 target_link_libraries(test_map_analysis
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   analysis
 )
if(OpenMP_CXX_FOUND)
 target_link_libraries(bag_map_analysis
   mesh
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   analysis
   OpenMP::OpenMP_CXX
 )
else() 
 target_link_libraries(bag_map_analysis
   mesh
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   analysis
 )
endif()


#############
## Install ##
#############

## Mark executables for installation
 install(TARGETS ${PROJECT_NAME} live_map_analysis test_map_analysis bag_map_analysis
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark libraries for installation
install(TARGETS analysis mesh
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

