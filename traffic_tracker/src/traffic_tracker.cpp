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
int bbox_no;
bool init_tracker = false;
cv::Rect2d roi;
cv::Mat frame;
long int min_x;
long int min_y;
long int max_x;
long int max_y;
long int x_dimension_bbox;
long int y_dimension_bbox;

cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();

int main(int arg, char **argv){

  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(10);

  cv::namedWindow("OpenCV_Feed");
  cv::startWindowThread();
  image_transport::ImageTransport transport_image(tt_nh);

  //ros::Subscriber tt_image_sub = tt_nh.subscribe("/darknet_ros/detection_image", 1000, extract_detection_image);

  // debugging subscriber (subscribes to video_file.launch)
  ros::Subscriber tt_image_sub = tt_nh.subscribe("/videofile/image_raw", 1000, extract_detection_image);

  ros::Subscriber tt_bbox_sub = tt_nh.subscribe("/darknet_ros/bounding_boxes", 1000, extract_bbox);
  // ros::Subscriber tt_count_sub = tt_nh.subscribe("/darknet_ros/found_object", 1000, extract_count);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();

    // does initializing tracker need to be inside of while loop?

  }

}

void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image){

  try{
    cv_bridge::CvImageConstPtr take_cv;
    take_cv = cv_bridge::toCvCopy(detection_image);
    frame = take_cv->image;

    if(init_tracker == false){
      roi = cv::Rect(min_x, min_y, x_dimension_bbox, y_dimension_bbox);
      tracker->init(frame, roi);
      init_tracker = true;
    }
    else{
      tracker->update(frame, roi);
      cv::rectangle(frame, roi, cv::Scalar(255,0,0),2,1);
      cv::imshow("OpenCV_Feed", frame);
      cv::waitKey(30);
    }
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Error - cannot launch OpenCV", detection_image->encoding.c_str());
  }

}

void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){
  min_x = bbox->bounding_boxes[0].xmin;
  min_y = bbox->bounding_boxes[0].ymin;
  max_x = bbox->bounding_boxes[0].xmax;
  max_y = bbox->bounding_boxes[0].ymax;
  x_dimension_bbox = (max_x - min_x);
  y_dimension_bbox = (max_y - min_y);
}
//
// void extract_count(const std_msgs::Int8::ConstPtr& count_value){
//
//   bbox_no = count_value->data;
//   init_tracker = true;
//
// }
