#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>

// function Prototypes
void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image);
// void extract_bounding_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);

int main(int arg, char **argv){

  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(10);

  // Ptr<Tracker> tracker = TrackerKCF::create();

  cv::namedWindow("OpenCV_Feed");
  cv::startWindowThread();
  image_transport::ImageTransport transport_image(tt_nh);
  ros::Subscriber tt_image_sub = tt_nh.subscribe("/darknet_ros/detection_image", 1000, extract_detection_image);

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image){

  try{
    cv_bridge::CvImageConstPtr take_cv;
    take_cv = cv_bridge::toCvCopy(detection_image);
    cv::Mat frame = take_cv->image;
    cv::imshow("OpenCV_Feed", frame);
    cv::waitKey(30);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Error - cannot launch OpenCV", detection_image->encoding.c_str());
  }

}
