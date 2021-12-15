#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omnimapper_msgs/StampedImageAndInfo.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

bool projectLocalPointRect(const omnimapper_msgs::StampedImageAndInfo& image, 
                           const tf::Point& pt, tf::Point & p);

sensor_msgs::Image MarkImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const tf::StampedTransform& cloud_transform,
                             const tf::StampedTransform& image_transform,
                             const omnimapper_msgs::StampedImageAndInfo& image);
