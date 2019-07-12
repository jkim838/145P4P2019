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
#include <string>
#include <sstream>
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
std::vector<cv::Point> trajectory_points;
std::vector<vehicle> vehicles; // vehicles and their coordinates in single frame
std::vector<vehicle> prev_vehicles; // vehicles and their coordinates in previous frame

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

    std::ofstream export_csv;
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
    export_csv << "==========\n";
    export_csv << "Frame ID:," << frame_count <<"\n";
    export_csv.close();

    if(frame_count == 1){
      prev_vehicles = vehicles;
    }
    else{
      // CASE 1:
      if(vehicles.size() == 0){
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "WARNING: No data in vector: vehicles\n";
        export_csv.close();
      }
      else{
        // force ID Rearrangement?
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "vehicle is " << vehicles.size() << " wide\n";
        export_csv << "list of elements: \n";
        export_csv << "ID:";
        for(size_t i = 0; i < vehicles.size(); i++){
          int current_x = vehicles[i].x;
          int current_y = vehicles[i].y;
          std::string current_class = vehicles[i].vehicle_class;
          vehicles[i] = {i, current_class, current_x, current_y};
          export_csv << vehicles[i].detection_ID << ",";
        }
        export_csv << "\n";
        export_csv << "X:";
        for(size_t i = 0; i < vehicles.size(); i++){
          export_csv << vehicles[i].x << ",";
        }
        export_csv << "\n";
        export_csv << "Y:";
        for(size_t i = 0; i < vehicles.size(); i++){
          export_csv << vehicles[i].y << ",";
        }
        export_csv << "\n";
        export_csv.close();
      }

      if(prev_vehicles.size() == 0){
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "WARNING: No data in vector: prev_vehicles\n";
        export_csv << "----------\n";
        export_csv.close();
      }
      else{
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "prev_vehicle is " << prev_vehicles.size() << " wide\n";
        export_csv << "list of elements: \n";
        export_csv << "ID:";
        for(size_t i = 0; i < prev_vehicles.size(); i++){
          export_csv << prev_vehicles[i].detection_ID << ",";
        }
        export_csv << "\n";
        export_csv << "X:";
        for(size_t i = 0; i < prev_vehicles.size(); i++){
          export_csv << prev_vehicles[i].x << ",";
        }
        export_csv << "\n";
        export_csv << "Y:";
        for(size_t i = 0; i < prev_vehicles.size(); i++){
          export_csv << prev_vehicles[i].y << ",";
        }
        export_csv << "\n";
        export_csv << "----------\n";
        export_csv.close();
      }

      /***
      IDEA: IF NO DATA IS IN VEHICLES, (hence tracker code won't run), then force previous frame into current frame to maintain?
      ***/
      for(size_t i = 0; i < vehicles.size(); i++){
        bool match_found = false; //determine if match was found for current element in previous frame...
        int current_x = vehicles[i].x; //hold to current x coordinate
        int current_y = vehicles[i].y; //hold to current y coordinate
        std::string current_class = vehicles[i].vehicle_class; //hold on to current class

        // BUG: This wouldn't run if there's no data in prev_vehicle
        for(size_t j = 0; j < prev_vehicles.size(); j++){
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "Comparing element "<< vehicles[i].detection_ID << " in current frame to element " << prev_vehicles[j].detection_ID << " in the previous frame\n";
          export_csv.close();
          int abs_x_diff = abs(vehicles[i].x - prev_vehicles[j].x);
          int abs_y_diff = abs(vehicles[i].y - prev_vehicles[j].y);
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "Current element: " << vehicles[i].detection_ID << "\n";
          export_csv << "X: " << current_x << "\n";
          export_csv << "Y: " << current_y << "\n";
          export_csv << "Previous element: " << prev_vehicles[j].detection_ID << "\n";
          export_csv << "X: " << prev_vehicles[j].x << "\n";
          export_csv << "X: " << prev_vehicles[j].y << "\n";
          export_csv << "Absolute difference in X:," << abs_x_diff <<"\n";
          export_csv << "Absolute difference in Y:," << abs_y_diff <<"\n";
          export_csv << "----------\n";
          export_csv.close();
          if(abs_x_diff < 100 && abs_y_diff < 100){
            // a match between new frame element to previous frame has been found.
            // assign to the new frame element the unique ID the matching element of the previous frame
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "Match between " << vehicles[i].detection_ID << " and " << prev_vehicles[j].detection_ID <<"\n";
            export_csv << "Reassigning Unique ID:" << prev_vehicles[j].detection_ID << " to current element, " << vehicles[i].detection_ID <<"\n";
            export_csv << "----------\n";
            export_csv.close();
            vehicles[i] = {prev_vehicles[j].detection_ID, prev_vehicles[j].vehicle_class, current_x, current_y}; //give current element unique ID and class of the matching previous frame
            match_found = true; // match was found for this element in previous frame (i.e. not a new vehicle)
          }
        }
        // BUG: when there is no detection in previous frame, the program will not execute following code and therefore no alteration to unique ID will be made
        if(!match_found && prev_vehicles.size() != 0){
          // if no match for current element is found in previous frame, this is a new vehicle entering the scene,
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "No match found for the element: " << vehicles[i].detection_ID <<"\n";
          export_csv.close();
          int maximum_ID = 0;
          for(int i = 0; i < prev_vehicles.size(); i++){
            if(maximum_ID < prev_vehicles[i].detection_ID){
              maximum_ID = prev_vehicles[i].detection_ID;
            }
          }
          int newUniqueID = maximum_ID + 1;
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "Assigning a new Unique ID:" << newUniqueID << " to current element, " << vehicles[i].detection_ID <<"\n";
          export_csv.close();
          vehicles[i] = {newUniqueID, current_class, current_x, current_y};
        }
        else if(!match_found && prev_vehicles.size() == 0){

        }

      }
      export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
      export_csv << "----------\n";
      export_csv << "Finished tracking\n";
      export_csv.close();
      // try drawing a heatmap for current frame
      for(size_t i = 0; i < vehicles.size(); i++){
        cv::Point center_point = cv::Point(vehicles[i].x, vehicles[i].y);
        cv::circle(frame, center_point, 10, cv::Scalar(255,0,0), 2, 1);
        std::stringstream toString;
        toString << vehicles[i].detection_ID;
        cv::putText(frame, toString.str(), center_point, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,255),2);
      }

      ROS_INFO("-----\n");
      ROS_INFO("Frame: %d\n", frame_count);
      export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
      export_csv << "Listing detected vehicles in frame: " << frame_count << "\n";
      export_csv.close();
      for(size_t i = 0; i < vehicles.size(); i++){
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "UniqueID:" << vehicles[i].detection_ID <<"\n";
        export_csv << "X: " << vehicles[i].x <<"\n";
        export_csv << "Y: " << vehicles[i].y <<"\n";
        export_csv.close();
        ROS_INFO("Unique ID: %d\n", vehicles[i].detection_ID);
        ROS_INFO("X: %d\n", vehicles[i].x);
        ROS_INFO("Y: %d\n", vehicles[i].y);
      }
      if(vehicles.size() != 0){
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "----------\n";
        export_csv << "Overwriting current frame vehicles to previous vehicles\n";
        export_csv.close();
        prev_vehicles = vehicles; // only overwrite when new vehicles infos are present
      }

    }
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
    export_csv << "----------\n";
    export_csv << "Wiping current frame vehicles\n";
    export_csv.close();
    vehicles.clear();
    frame_count++;
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Error - cannot launch OpenCV", detection_image->encoding.c_str());
  }
  cv::namedWindow("Tracker", CV_WINDOW_AUTOSIZE);
  cv::imshow("Tracker", frame);
  cv::waitKey(30);

}

void extract_bbox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){
  // HINT: index i does not always represent detected object of interest...
  // i.e. if detected object class was a person, that 'i' is useless
  // setup a id counter for vehicles only.
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
    if(vehicle_class=="bicycle"||vehicle_class=="car"||vehicle_class=="motorbike"||vehicle_class=="bus"||vehicle_class=="truck"){
        vehicle current = {i, vehicle_class, x_center, y_center};
        vehicles.push_back(current);
    }
  }

}
//
void extract_count(const std_msgs::Int8::ConstPtr& count_value){

    bbox_no = count_value->data;

}
