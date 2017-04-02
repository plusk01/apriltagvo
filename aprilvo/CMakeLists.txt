cmake_minimum_required(VERSION 2.8.3)
project(aprilvo)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
	image_transport
	cv_bridge
	sensor_msgs
  nav_msgs
	message_generation
  tf
  image_geometry

  # AprilTags library from MIT
  apriltags
)

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
include_directories(include/aprilvo ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# AprilVO Node
add_executable(aprilvo src/AprilVO_node.cpp src/AprilVO.cpp)
add_dependencies(aprilvo aprilvo_generate_messages_cpp)
target_link_libraries(aprilvo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})