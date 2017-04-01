#include <ros/ros.h>

#include "AprilVO.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aprilvo_node");
  AprilVO thing;

  ros::spin();
  return 0;
}
