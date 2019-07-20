#include "traffic_tracker_additional_functions.cpp"

int main(int arg, char **argv){
  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(1000);

  image_transport::ImageTransport transport_image(tt_nh);

  /*** SUBSCRIBER DEFINITIONS ***/
  ros::Subscriber tt_bbox_sub = tt_nh.subscribe("/darknet_ros/bounding_boxes",
  1000, extract_bbox);
  #ifdef SUB_RAW_FEED
    ros::Subscriber tt_image_sub = tt_nh.subscribe("/videofile/image_raw",1000,
    extract_detection_image);
  #else
    ros::Subscriber tt_image_sub = tt_nh.subscribe(
    "/darknet_ros/detection_image", 1000, extract_detection_image);
  #endif

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  frame.release();
}

void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image){
  getFrameFromSource(detection_image);

  #ifdef ENABLE_DEBUG_MODE
  debugListFrame();
  #endif

  preprocessVehicles();
  beginTracking();

  #ifdef ENABLE_DEBUG_MODE
  debugListVehicle();
  #endif

  prepareNextFrame();
  frame_count++;
  displayFeed();
}

void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){
  getBBOXinfo(bbox);
}
