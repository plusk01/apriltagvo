#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag36h11.h>

static const std::string OPENCV_WINDOW = "Image window";

class AprilTagVO
{
public:
  AprilTagVO();
  ~AprilTagVO();

  seer::AprilTag convert_to_msg(AprilTags::TagDetection& detection, int width, int height);

  void processCvImage(cv_bridge::CvImagePtr cv_ptr);

  void imageCb(const sensor_msgs::ImageConstPtr& msg);

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher tag_list_pub;
  AprilTags::TagDetector* tag_detector;

  // allow configurations for these:  
  AprilTags::TagCodes tag_codes;
  double camera_focal_length_x; // in pixels. late 2013 macbookpro retina = 700
  double camera_focal_length_y; // in pixels
  double tag_size; // tag side length of frame in meters 
  bool  show_debug_image;
};