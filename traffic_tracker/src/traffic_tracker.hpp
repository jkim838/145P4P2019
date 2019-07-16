/*** ROS ***/
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

/*** CPP ***/
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <string>
#include <sstream>
#include <math.h>
#include <algorithm>

/*** OpenCV ***/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>

/*** Macro Definitions ***/
//#define ENABLE_DEBUG_MODE

/*** Function Prototypes ***/
void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image);
void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void extract_count(const std_msgs::Int8::ConstPtr& count_value);

/*** Struct Definitions ***/
// define each instance of vehicle in the frame
struct vehicle{
  int detection_ID; // ID assigned by object_detector
  std::string vehicle_class; // type of vehicle
  long int x; // center coordinate x
  long int y; // center coordinate y
  long int x_dimension;
  long int y_dimension;
};

// define an instance of checkpoint inthe frame
struct checkpoint{
  std::string VRI;
  long int timestamp;
};

// define each instance of tracked vehicle after processing
struct tracked_vehicle{
  int unique_ID;
  std::string vehicle_class;
  float velocity;
  std::vector<checkpoint> checkpoints;
};

// OpenCV Related Global Variables
cv::Mat frame;
cv::Point center_point;
std::vector<cv::Point> trajectory_points;
std::vector<vehicle> vehicles; // vehicles and their coordinates in single frame
std::vector<vehicle> prev_vehicles; // vehicles and their coordinates in previous frame

// Tracker Related Global Variables
int maximum_ID = 0;
int bbox_no = 0;
long int frame_count = 1; //first frame number
