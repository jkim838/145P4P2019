#include <ros/ros.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Int8.h>

// Structure Objects
struct vehicle{
  int detection_ID; // ID assigned by object_detector
  std::string vehicle_class; // type of vehicle
  long int x; // center coordinate x
  long int y; // center coordinate y
};

struct tracked_vehicle{
  int unique_ID;
  std::string Class;
  long int x;
  long int y;
};

// function Prototypes
void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image);
void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void extract_count(const std_msgs::Int8::ConstPtr& count_value);

// global variables and prototypes
cv::Mat frame;
cv::Point center_point;
long int frame_count = 1;
long int bbox_count = 0;
int bbox_no = 0;
int giveUniqueID = 0;
//std::vector<cv::Rect> ROIs;
std::vector<cv::Point> trajectory_points;
std::vector<vehicle> vehicles; // vehicles and their coordinates in single frame
std::vector<vehicle> prev_vehicles; // vehicles and their coordinates in previous frame
std::vector<tracked_vehicle> tracked_vehicles;
vehicle current;


int main(int arg, char **argv){

  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(1000);

  image_transport::ImageTransport transport_image(tt_nh);
  ros::Subscriber tt_bbox_sub = tt_nh.subscribe("/darknet_ros/bounding_boxes", 1000, extract_bbox);
  ros::Subscriber tt_image_sub = tt_nh.subscribe("/darknet_ros/detection_image", 1000, extract_detection_image);
  //debugging subscriber (subscribes to video_file.launch)
  //ros::Subscriber tt_image_sub = tt_nh.subscribe("/videofile/image_raw", 1000, extract_detection_image);
  ros::Subscriber tt_count_sub = tt_nh.subscribe("/darknet_ros/found_object", 1000, extract_count);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  frame.release();

}

void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image){

  try{
    cv_bridge::CvImageConstPtr take_cv;
    take_cv = cv_bridge::toCvCopy(detection_image);
    frame = take_cv->image;
    if(frame_count == 1){
      prev_vehicles = vehicles;
    }
    else{
      for(int i = 0; i < prev_vehicles.size(); i++){
        for(int j = 0; j < vehicles.size(); j++){
          int abs_x_diff = abs(prev_vehicles[i].x - vehicles[j].x);
          int abs_y_diff = abs(prev_vehicles[i].y - vehicles[j].y);
          int x_coord = vehicles[j].x;
          int y_coord = vehicles[j].y;
          std::string Class = vehicles[j].vehicle_class;
          if(abs_x_diff < 20 && abs_y_diff < 20){
            vehicles[j] = {prev_vehicles[i].detection_ID, prev_vehicles[i].vehicle_class, x_coord, y_coord};
          }
          else{
            // new vehicle on the scene,
            vehicles[j] = {prev_vehicles.size()+1, Class, x_coord, y_coord};
          }
        }
      }
      ROS_INFO("-----\n");
      ROS_INFO("Frame: %d\n", frame_count);
      std::ofstream export_csv;
      export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
      export_csv << "Frame ID:," << frame_count <<"\n";
      export_csv.close();
      for(int i = 0; i < vehicles.size(); i++){
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "UniqueID:," << vehicles[i].detection_ID <<"\n";
        export_csv << "X:," << vehicles[i].x <<"\n";
        export_csv << "Y:," << vehicles[i].y <<"\n";
        export_csv.close();
        ROS_INFO("Unique ID: %d\n", vehicles[i].detection_ID);
        ROS_INFO("X: %d\n", vehicles[i].x);
        ROS_INFO("Y: %d\n", vehicles[i].y);
      }
      prev_vehicles = vehicles;
    }
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Error - cannot launch OpenCV", detection_image->encoding.c_str());
  }
  frame_count++;
  vehicles.clear();
  cv::namedWindow("Tracker", CV_WINDOW_AUTOSIZE);
  cv::imshow("Tracker", frame);
  cv::waitKey(30);

}

void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){

  for(int i = 0; i < bbox->bounding_boxes.size(); i++){
    long int min_x = bbox->bounding_boxes[i].xmin;
    long int min_y = bbox->bounding_boxes[i].ymin;
    long int max_x = bbox->bounding_boxes[i].xmax;
    long int max_y = bbox->bounding_boxes[i].ymax;
    std::string vehicle_class = bbox->bounding_boxes[i].Class;
    long int x_dimension_bbox = (max_x - min_x);
    long int y_dimension_bbox = (max_y - min_y);
    long int x_center = min_x + x_dimension_bbox/2;
    long int y_center = min_y + y_dimension_bbox/2;
    current = {i, vehicle_class, x_center, y_center};
    vehicles.push_back(current);
  }

}
//
void extract_count(const std_msgs::Int8::ConstPtr& count_value){

    bbox_no = count_value->data;

}
