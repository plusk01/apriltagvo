#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

#include "aprilvo/AprilTag.h"
#include "aprilvo/AprilTagList.h"


class AprilVO
{
public:
  AprilVO();
  ~AprilVO();

private:
  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // image transport pub/sub
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;

  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;

  // ROS publishers and subscribers
  ros::Publisher tag_list_pub;
  ros::Publisher estimate_pub;

  AprilTags::TagDetector* tag_detector;

  // allow configurations for these:  
  AprilTags::TagCodes tag_codes;
  double tag_size; // tag side length of frame in meters 
  bool  show_output_video;

  double standardRad(double t);
  void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);

  void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo);

  aprilvo::AprilTag convert_to_msg(AprilTags::TagDetection& detection);

  void processCvImage(cv_bridge::CvImagePtr cv_ptr);
};