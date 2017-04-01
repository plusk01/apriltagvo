#include <ros/ros.h>

#include "AprilTagVO.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltagvo_node");
  AprilTagVO thing;

  ros::spin();
  return 0;
}
