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
#include <cstring>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Int8.h>

// function Prototypes
void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image);
void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void extract_count(const std_msgs::Int8::ConstPtr& count_value);

// global variables and prototypes
cv::Mat frame;
cv::Point center_point;
long int frame_count = 0;
long int bbox_count = 0;
int bbox_no = 0;
//std::vector<cv::Rect> ROIs;
std::vector<cv::Point> trajectory_points;

int main(int arg, char **argv){

  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(1000);

  image_transport::ImageTransport transport_image(tt_nh);
  ros::Subscriber tt_bbox_sub = tt_nh.subscribe("/darknet_ros/bounding_boxes", 1000, extract_bbox);
  //ros::Subscriber tt_image_sub = tt_nh.subscribe("/darknet_ros/detection_image", 1000, extract_detection_image);
  //debugging subscriber (subscribes to video_file.launch)
  ros::Subscriber tt_image_sub = tt_nh.subscribe("/videofile/image_raw", 1000, extract_detection_image);
  ros::Subscriber tt_count_sub = tt_nh.subscribe("/darknet_ros/found_object", 1000, extract_count);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();

    // does initializing tracker need to be inside of while loop?

  }
  frame.release();

}

void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image){

  try{
    cv_bridge::CvImageConstPtr take_cv;
    take_cv = cv_bridge::toCvCopy(detection_image);
    frame = take_cv->image;
    for(size_t i = 0; i < trajectory_points.size(); i++){
      cv::circle(frame, trajectory_points[i], 3, cv::Scalar(0,0,255), 2, 1);
    }
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Error - cannot launch OpenCV", detection_image->encoding.c_str());
  }
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
    long int x_dimension_bbox = (max_x - min_x);
    long int y_dimension_bbox = (max_y - min_y);
    long int x_center = min_x + x_dimension_bbox/2;
    long int y_center = min_y + y_dimension_bbox/2;
    center_point = cv::Point(x_center, y_center);
    trajectory_points.push_back(center_point);
  }

}
//
void extract_count(const std_msgs::Int8::ConstPtr& count_value){

    bbox_no = count_value->data;

}
