#include <ros/ros.h>

#include "AprilTagVO.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aprilvo_node");
  AprilTagVO thing;

  ros::spin();
  return 0;
}
