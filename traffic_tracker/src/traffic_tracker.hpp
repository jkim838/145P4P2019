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
#include <chrono>
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
/*** Others ***/


/*** Macro Definitions ***/
#define ENABLE_DEBUG_MODE
#define ROI_DEBUG_MODE
//#define SUB_RAW_FEED

/*** Function Prototypes ***/
void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image);
void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void extract_count(const std_msgs::Int8::ConstPtr& count_value);
void initialize_vri();
void getBBOXinfo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void getFrameFromSource(const sensor_msgs::Image::ConstPtr& detection_image);
void displayFeed();
void preprocessVehicles();
void beginTracking();
void prepareNextFrame();
void generatePerspective();
void debugListVehicle();
void debugListFrame();

/*** Struct Definitions ***/
// define each instance of vehicle in the frame
struct vehicle{
  int detectionID; // ID assigned by object_detector
  std::string vehicleClass; // type of vehicle
  long int x; // center coordinate x
  long int y; // center coordinate y
  long int xDimension;
  long int yDimension;
};

// define each instance of tracked vehicle after processing
struct tracked_vehicle{
  int uniqueID;
  std::string vehicleClass;
  std::vector<unsigned long> timestamps;
  std::vector<int> checkpoints;
  long int frameInit; // frame which vehicle was initialized at
};

struct msg_vehicle{
  int unique_ID; // Unique ID of the vehicle to be passed down as ROS MSG
  float velocity; // velocity of the vehicle to be passed down as ROS MSG
  std::string Class; // vehicle type to be passed down as ROS MSG
  std::vector<int> checkpoints; // list of checkpoints passed by vehicle as ROS MSG
};

// OpenCV Related Global Variables
cv::Mat frame;

// Tracker Related Global Variables
int cp_begin_y = 865;
int cp_end_y = 310;
int cp_quantity = 1;
std::vector<int> cp_coords_y;

std::vector<vehicle> vehicles; // vehicles and their coordinates in single frame
std::vector<vehicle> prev_vehicles; // vehicles and their coordinates in previous frame
std::vector<int> TrackingIDs; // list of IDs of vehicles that are currently being tracked
std::vector<tracked_vehicle> TrackingVehicles; // list of vehicles that are currently being tracked
std::vector<msg_vehicle> MsgVehicles; // list of vehicles to be passed down as ROS MSG

// Tracker Related Global Variables
int maximum_ID = 0;
int bbox_no = 0;
long int frame_count = 1; //first frame number

#ifdef ENABLE_DEBUG_MODE
  std::ofstream export_csv;
#endif

/*** Function Definitions ***/
void initialize_vri()
{
  cp_coords_y.push_back(cp_begin_y);
  for(int i = 0; i < cp_quantity; i++){
    int cp_total_distance = std::max(cp_begin_y, cp_end_y) - std::min(cp_begin_y, cp_end_y); // total distance between the beginning and the end of the checkpoint
    int coords_to_push = cp_begin_y - (cp_total_distance / (cp_quantity+1)) * (i + 1);
    cp_coords_y.push_back(coords_to_push);
  }
  cp_coords_y.push_back(cp_end_y);
}
