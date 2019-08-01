#include "traffic_tracker_additional_functions.cpp"

int main(int arg, char **argv){
  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(1000);

  image_transport::ImageTransport transport_image(tt_nh);

  /*** SUBSCRIBER DEFINITIONS ***/
  ros::Subscriber tt_bbox_sub = tt_nh.subscribe("/darknet_ros/bounding_boxes",
  1000, extract_bbox);
  tt_tracker_pub = tt_nh.advertise<traffic_tracker::trackerOutput>
  ("/traffic_tracker/trackerOutput", 1000);
  #ifdef SUB_RAW_FEED
    ros::Subscriber tt_image_sub = tt_nh.subscribe("/videofile/image_raw",50,
    extract_detection_image);
  #else
    ros::Subscriber tt_image_sub = tt_nh.subscribe(
    "/darknet_ros/detection_image", 50, extract_detection_image);
  #endif

  #ifdef ENABLE_PERSPECTIVE_FEED
    cv::namedWindow("Perspective", CV_WINDOW_NORMAL);
    cv::moveWindow("Perspective", 0,0);
    cv::resizeWindow("Perspective", 1280, 720);
  #endif
  #ifdef ENABLE_TRACKER_FEED
    cv::namedWindow("Tracker", CV_WINDOW_NORMAL);
    cv::moveWindow("Tracker", 0,0);
    cv::resizeWindow("Tracker", 1280, 720);
  #endif
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    frame_count++;
  }
  frame.release();
}

void extract_detection_image(const sensor_msgs::Image::ConstPtr& detection_image){
  getFrameFromSource(detection_image);

  #ifdef ENABLE_DEBUG_MODE
  debugListFrame();
  #endif

  #ifdef ENABLE_MOTION_TRACKING
    preprocessVehicles();

    #ifdef ENABLE_PERSPECTIVE_TRACKING
    generatePerspective();
    #endif

    beginTracking();

    #ifdef ENABLE_PERSPECTIVE_TRACKING
    extractPerspectiveCoord();
    #endif

    #ifdef ENABLE_DEBUG_MODE
    debugListVehicle();
    #endif
  #endif

  #ifdef ENABLE_TRACKER_FEED
  displayFeed("Tracker", frame);
  #endif

  #ifdef ENABLE_MOTION_TRACKING
    #ifdef ENABLE_PERSPECTIVE_FEED
    displayFeed("Perspective", ppImage);
    #endif

    prepareNextFrame();
  #endif
}

void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){
  #ifdef ENABLE_MOTION_TRACKING
  getBBOXinfo(bbox);
  #endif
}
