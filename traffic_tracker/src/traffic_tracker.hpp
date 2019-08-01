/*** ROS ***/
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <traffic_tracker/perspectiveVehicle.h>
#include <traffic_tracker/trackerOutput.h>
#include <traffic_tracker/point2f.h>

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
#define ENABLE_MOTION_TRACKING
#define ENABLE_PERSPECTIVE_TRACKING
#define ENABLE_PERSPECTIVE_FEED
#define DRAW_PERSPECTIVE_INFO
//#define ENABLE_TRACKER_FEED
//#define DRAW_TRACKER_INFO
//#define SUB_RAW_FEED

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

struct perspectiveVehicle{
  int uniqueID;
  std::string vehicleClass;
  std::vector<cv::Point2f> centerPoint;
  std::vector<long int> frameNo;
};

/*** Function Prototypes ***/
void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image);
void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void extract_count(const std_msgs::Int8::ConstPtr& count_value);
void initialize_vri();
void getBBOXinfo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void getFrameFromSource(const sensor_msgs::Image::ConstPtr& detection_image);
void displayFeed(std::string windowName, cv::Mat imageName);
void preprocessVehicles();
void beginTracking();
void prepareNextFrame();
void generatePerspective();
void extractPerspectiveCoord();
void debugListVehicle();
void debugListFrame();

// OpenCV Related Global Variables
cv::Mat frame;
cv::Mat ppImage;
cv::Mat ppMatrix;
cv::Size ppImageSize = cv::Size(690, 1220);
std::vector<cv::Point2f> ppCenterPointIn;
std::vector<cv::Point2f> ppCenterPointOut;
std::vector<perspectiveVehicle> ppVehicleFrame;


// Tracker Related Global Variables
std::vector<vehicle> vehicles; // vehicles and their coordinates in single frame
std::vector<vehicle> prev_vehicles; // vehicles and their coordinates in previous frame
std::vector<perspectiveVehicle> TrackedVehicles;
std::vector<int> EndTrackVehicles;

// Tracker Related Global Variables
int maximum_ID = 0;
int bbox_no = 0;
long int frame_count = 1; //first frame number
bool runPerspective = false;

// ROS Related Global variables
ros::Publisher tt_tracker_pub;
#ifdef ENABLE_DEBUG_MODE
  std::ofstream export_csv;
#endif
// ROS related global Variables
traffic_tracker::trackerOutput msg;
