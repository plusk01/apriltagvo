#include "AprilVO.h"

static const std::string OPENCV_WINDOW = "Image window";

// ----------------------------------------------------------------------------

AprilVO::AprilVO() :
  nh_(ros::NodeHandle()),
  nh_private_("~"),
  it_(nh_), 
  tag_codes(AprilTags::tagCodes36h11), 
  tag_detector(NULL),
  tag_size(0.029), // 1 1/8in marker = 0.029m
  show_output_video(false)
{
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  nh_private_.param<double>("tag_size_m", tag_size, 0.029);
  nh_private_.param<bool>("show_output_video", show_output_video, false);

  // Create ROS publishers
  tag_list_pub = nh_.advertise<aprilvo::AprilTagList>("apriltags", 100);
  estimate_pub = nh_.advertise<nav_msgs::Odometry>("estimate", 1);

  // Subscribe to input video feed and publish output video feed
  it_ = image_transport::ImageTransport(nh_);
  image_sub_ = it_.subscribeCamera("input_image", 1, &AprilVO::cameraCallback, this);
  image_pub_ = it_.advertise("output_image", 1);

  // Create a new April Tag detector from the library
  tag_detector = new AprilTags::TagDetector(tag_codes);

  // Show the output video for debugging
  if (show_output_video)
    cv::namedWindow(OPENCV_WINDOW);
}

// ----------------------------------------------------------------------------

AprilVO::~AprilVO() {
  if (show_output_video)
    cv::destroyWindow(OPENCV_WINDOW);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

aprilvo::AprilTag AprilVO::convert_to_msg(AprilTags::TagDetection& detection) {
  // recovering the relative pose of a tag:

  // NOTE: for this to be accurate, it is necessary to use the
  // actual camera parameters here as well as the actual tag size
  // (m_fx, m_fy, m_px, m_py, m_tagSize)

  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detection.getRelativeTranslationRotation(tag_size, 
                                           cam_model_.fx(), 
                                           cam_model_.fy(), 
                                           cam_model_.cx(), 
                                           cam_model_.cy(),
                                           translation, 
                                           rotation);

  Eigen::Matrix3d F;
  F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
  Eigen::Matrix3d fixed_rot = F*rotation;
  double yaw, pitch, roll;
  wRo_to_euler(fixed_rot, yaw, pitch, roll);


  aprilvo::AprilTag tag_msg;

  tag_msg.id = detection.id;
  tag_msg.hamming_distance = detection.hammingDistance;
  tag_msg.distance = translation.norm() * 100.0;
  tag_msg.z = translation(0) * 100.0; // depth from camera
  tag_msg.x = translation(1) * 100.0; // horizontal displacement (camera pov right = +ve)
  tag_msg.y = translation(2) * 100.0; // vertical displacement
  tag_msg.yaw = yaw;
  tag_msg.pitch = pitch;
  tag_msg.roll = roll;
  return tag_msg;
}

// ----------------------------------------------------------------------------

void AprilVO::cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // update the camera model with the camera's intrinsic parameters
  cam_model_.fromCameraInfo(cinfo);

  processCvImage(cv_ptr);

  if (show_output_video) {
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

// ----------------------------------------------------------------------------

void AprilVO::processCvImage(cv_bridge::CvImagePtr cv_ptr)  {
  cv::Mat image_gray;
  cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
  vector<AprilTags::TagDetection> detections = tag_detector->extractTags(image_gray);
  vector<aprilvo::AprilTag> tag_msgs;

  for (int i=0; i<detections.size(); i++) {
    detections[i].draw(cv_ptr->image);
    tag_msgs.push_back(convert_to_msg(detections[i]));
  }

  if(detections.size() > 0) { // take this out if you want absence notificaiton
    aprilvo::AprilTagList tag_list;
    tag_list.april_tags = tag_msgs;
    tag_list_pub.publish(tag_list);
  }
}

// ----------------------------------------------------------------------------

void AprilVO::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

// ----------------------------------------------------------------------------

double AprilVO::standardRad(double t) {

  // normalize the angle to be within the interval [-pi, pi]
  if (t >= 0.0) {
    t = fmod(t+M_PI, 2*M_PI) - M_PI;
  } else {
    t = fmod(t-M_PI, -2*M_PI) + M_PI;
  }

  return t;
}

// ----------------------------------------------------------------------------