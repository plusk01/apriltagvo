cmake_minimum_required(VERSION 2.8.3)
project(apriltagvo)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
	image_transport
	cv_bridge
	sensor_msgs
	message_generation
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  AprilTag.msg
  AprilTagList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 # INCLUDE_DIRS include include/AprilTags
 # LIBRARIES aprilVO
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/AprilTags ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ library
add_library(apriltags
  src/AprilTags/Edge.cc
  src/AprilTags/FloatImage.cc
  src/AprilTags/GLine2D.cc
  src/AprilTags/GLineSegment2D.cc
  src/AprilTags/Gaussian.cc
  src/AprilTags/GrayModel.cc
  src/AprilTags/Homography33.cc
  src/AprilTags/MathUtil.cc
  src/AprilTags/Quad.cc
  src/AprilTags/Segment.cc
  src/AprilTags/TagDetection.cc
  src/AprilTags/TagDetector.cc
  src/AprilTags/TagFamily.cc
  src/AprilTags/UnionFindSimple.cc
)

target_link_libraries(apriltags
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# AprilTagVO Node
add_executable(apriltagvo src/AprilTagVO_node.cpp src/AprilTagVO.cpp)
add_dependencies(apriltagvo apriltagvo_generate_messages_cpp)
target_link_libraries(apriltagvo apriltags ${catkin_LIBRARIES})