#include "traffic_tracker.hpp"

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

    /*** BEGIN DEBUG MESSAGE ***/
    #ifdef ENABLE_DEBUG_MODE
      std::ofstream export_csv;
      export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
      export_csv << "==========\n";
      export_csv << "Frame ID:," << frame_count <<"\n";
      export_csv.close();
    #endif
    /*** END DEBUG MESSAGE ***/

    if(frame_count == 1){
      for(int i = 0; i < vehicles.size(); i++){
        long int current_x = vehicles[i].x;
        long int current_y = vehicles[i].y;
        std::string current_class = vehicles[i].vehicle_class;
        long int current_ID = vehicles[i].detection_ID;
        long int current_x_dimension = vehicles[i].x_dimension;
        long int current_y_dimension = vehicles[i].y_dimension;
        prev_vehicles[i] = {current_ID, current_class, current_x, current_y, current_x_dimension, current_y_dimension};
      }
    }
    else{
      // CASE 1:
      if(vehicles.size() == 0){

        /*** BEGIN DEBUG MESSAGE ***/
        #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "WARNING: No data in vector: vehicles\n";
          export_csv.close();
        #endif
        /*** END DEBUG MESSAGE ***/
      }
      else{
        // force ID Rearrangement?
        for(int i = 0; i < vehicles.size(); i++){
          long int current_x = vehicles[i].x;
          long int current_y = vehicles[i].y;
          std::string current_class = vehicles[i].vehicle_class;
          long int current_x_dimension = vehicles[i].x_dimension;
          long int current_y_dimension = vehicles[i].y_dimension;
          vehicles[i] = {i, current_class, current_x, current_y, current_x_dimension, current_y_dimension};
        }

        /***BEGIN DEBUG MESSAGE ***/
        #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "vehicle is " << vehicles.size() << " wide\n";
          export_csv << "list of elements: \n";
          export_csv << "ID:";
          for(size_t i = 0; i < vehicles.size(); i++){
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
        #endif
        /***END DEBUG MESSAGE***/

      }

      /*** BEGIN DEBUG MESSAGE ***/
      #ifdef ENABLE_DEBUG_MODE
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
      #endif
      /*** END DEBUG MESSAGE ***/

      for(size_t i = 0; i < vehicles.size(); i++){

        bool match_found = false; //determine if match was found for current element in previous frame...
        long int current_x = vehicles[i].x; //hold to current x coordinate
        long int current_y = vehicles[i].y; //hold to current y coordinate
        std::string current_class = vehicles[i].vehicle_class; //hold on to current class
        long int current_x_dimension = vehicles[i].x_dimension;
        long int current_y_dimension = vehicles[i].y_dimension;

        long int current_x_min = current_x - (current_x_dimension / 2); //minimum x-coordinate value, x1
        long int current_x_max = current_x + (current_x_dimension / 2); //maximum x-coordinate value, x2
        long int current_y_min = current_y - (current_y_dimension / 2); //minimum y-coordinate value, y1
        long int current_y_max = current_y + (current_y_dimension / 2); //maximum y-coordinate value, y2
        long int area_current = (current_x_max - current_x_min) * (current_y_max - current_y_min); // calculate the area of bounding box in px...

        int iteration_count = 0;

        // BUG: This wouldn't run if there's no data in prev_vehicle
        for(size_t j = 0; j < prev_vehicles.size(); j++){

          long int prev_x = prev_vehicles[j].x;
          long int prev_y = prev_vehicles[j].y;
          long int prev_x_dimension = prev_vehicles[j].x_dimension;
          long int prev_y_dimension = prev_vehicles[j].y_dimension;

          long int previous_x_min = prev_x - (prev_x_dimension / 2); // minimum x-coordinate value, x1
          long int previous_x_max = prev_x + (prev_x_dimension / 2); // maximum x-coordinate value, x2
          long int previous_y_min = prev_y - (prev_y_dimension / 2); // minimum y-coordinate value, y1
          long int previous_y_max = prev_y + (prev_y_dimension / 2); // maximum y-coordinate value, y2

          long int intersection_x1 = std::max(current_x_min, previous_x_min); // minimum x-coordinate value of intersecting rectangle
          long int intersection_x2 = std::min(current_x_max, previous_x_max); // maximum x-coordinate value of the intersecting rectangle
          long int intersection_y1 = std::max(current_y_min, previous_y_min); // minimum y-coordinate of the intersecting rectangle
          long int intersection_y2 = std::min(current_y_max, previous_y_max); // maximum y-coordinate of the intersecting rectangle

          long int intersection_width = (intersection_x2 - intersection_x1);
          long int intersection_height = (intersection_y2 - intersection_y1);

          long int area_intersection;
          if(intersection_width < 0 || intersection_height < 0){
            area_intersection = 0; // for a case where two bounding boxes do not overlap...
          }
          else{
            area_intersection = intersection_width * intersection_height;
          }

          long int area_previous = (previous_y_max - previous_y_min) * (previous_x_max - previous_x_min);
          long int area_union = area_current + area_previous - area_intersection;


          float IOU = (float)area_intersection / (float)area_union;

          float IOU_Threshold = 0.35;
          int abs_x_diff = abs(vehicles[i].x - prev_vehicles[j].x);
          int abs_y_diff = abs(vehicles[i].y - prev_vehicles[j].y);
          int euclid_distance = hypot(abs_x_diff, abs_y_diff);
          int euclid_threshold = hypot(current_x_dimension, current_y_dimension)/6; // give strict Euclidean distance threshold as it is only a back-up threshold

          /*** BEGIN DEBUG MESSAGE ***/
          #ifdef ENABLE_DEBUG_MODE
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "Comparing element "<< vehicles[i].detection_ID << " in current frame to element " << prev_vehicles[j].detection_ID << " in the previous frame\n";
            export_csv << "Current element: " << vehicles[i].detection_ID << "\n";
            export_csv << "Center X: " << current_x << "\n";
            export_csv << "Center Y: " << current_y << "\n";
            export_csv << "X-Dimension: " << current_x_dimension << "\n";
            export_csv << "Y-Dimension: " << current_y_dimension << "\n";
            export_csv << "Area: " << current_y_dimension * current_x_dimension << "\n";
            export_csv << "----------\n";
            export_csv << "Previous element: " << prev_vehicles[j].detection_ID << "\n";
            export_csv << "Center X: " << prev_x << "\n";
            export_csv << "Center Y: " << prev_y << "\n";
            export_csv << "X-Dimension: " << prev_x_dimension << "\n";
            export_csv << "Y-Dimension: " << prev_y_dimension << "\n";
            export_csv << "Area: " << prev_y_dimension * prev_x_dimension << "\n";
            export_csv << "----------\n";
            export_csv << "Area of Intersection:" << area_intersection <<"\n";
            export_csv << "Area of Union:" << area_union <<"\n";
            export_csv << "Correlation: " << IOU <<"\n";
            export_csv << "Euclidean Distance: " << euclid_distance << "\n";
            export_csv << "----------\n";
            export_csv << "IOU Threshold: "<< IOU_Threshold << "\n";
            export_csv << "Euclid threshold value X: "<< euclid_threshold << "\n";
            export_csv << "----------\n";
            export_csv.close();
          #endif
          /***END DEBUG MESSAGE***/

          // TODO: DYNAMICALLY ADJUST THE THRESHOLD VALUE PER DIFFERENT SIZE OF BBOX...
          if(IOU > IOU_Threshold || euclid_distance < euclid_threshold){
            // a match between new frame element to previous frame has been found.
            // assign to the new frame element the unique ID the matching element of the previous frame

            /***BEGIN DEBUG MESSAGE ***/
            #ifdef ENABLE_DEBUG_MODE
              export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
              export_csv << "Match between " << vehicles[i].detection_ID << " and " << prev_vehicles[j].detection_ID <<"\n";
              export_csv << "Reassigning Unique ID:" << prev_vehicles[j].detection_ID << " to current element, " << vehicles[i].detection_ID <<"\n";
              export_csv << "----------\n";
              export_csv.close();
            #endif
            /*** END DEBUG MESSAGE ***/

            vehicles[i] = {prev_vehicles[j].detection_ID, prev_vehicles[j].vehicle_class, current_x, current_y, current_x_dimension, current_y_dimension};
            //give current element unique ID and class of the matching previous frame
            match_found = true; // match was found for this element in previous frame (i.e. not a new vehicle)

          }
          iteration_count++;

        }

        //Need to find the maximum ID EVER assigned in history of prev_vehicles...
        for(int i = 0; i < vehicles.size(); i++){
          if(maximum_ID < vehicles[i].detection_ID){
            maximum_ID = vehicles[i].detection_ID;
          }
        }

        if(!match_found && prev_vehicles.size() != 0 && iteration_count == prev_vehicles.size()){
          // if no match for current element is found in previous frame, this is a new vehicle entering the scene,

          /*** BEGIN DEBUG MESSAGE ***/
          #ifdef ENABLE_DEBUG_MODE
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "No match found for the element: " << vehicles[i].detection_ID <<"\n";
            export_csv.close();
          #endif
          /*** END DEBUG MESSAGE***/

          int newUniqueID = maximum_ID + 1;

          /*** BEGIN DEBUG MESSAGE ***/
          #ifdef ENABLE_DEBUG_MODE
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "Global maximum unique ID was: " << maximum_ID << "\n";
            export_csv << "Assigning a new Unique ID:" << newUniqueID << " to current element, " << vehicles[i].detection_ID <<"\n";
            export_csv.close();
          #endif
          /*** END DEBUG MESSAGE ***/

          vehicles[i] = {newUniqueID, current_class, current_x, current_y, current_x_dimension, current_y_dimension};
        }

      }

      /*** DEBUG: PUT VRI CODE HERE? ***/
      // What we need:
      //
      //      std::vector<long int> IDs_To_Track // list of unique IDs to track between ROIs...
      //
      //      struct tracked_vehicle{
      //        long int uniqueID; // unique ID of the vehicle to track
      //        std::string Class; // vehicle type
      //        std::vector<float> timestamps; // system time recorded every time vehicle enters the ROI
      //        std::vector<int> checkpoint_number; // list of ROIs the vehicle has passed through
      //      }
      //
      //      std::vector<tracked_vehicle> tracked_vehicles // list of tracked vehicles
      //
      //      struct msg_vehicle{
      //        long int uniqueID; // unique ID of vehicle transmitted as ROS msg
      //        float velocity; // velocity of the vehicle transmitted as ROS msg
      //        std::string Class; // class of vehicle transmitted as ROS msg
      //        std::vector<int> checkpoint_numbers // list of checkpoints vehicle traveled, as ROS msg
      //      }
      //
      //      std::vector<msg_vehicle> msg_vehicles // list of vehicles to be transmitted as ROS msg
      //
      //      int cp_begin_y // y-coordinate of the first checkpoint
      //
      //      int cp_end_y // y-coordinate of the last checkpoint
      //
      //      int cp_quantity // number of desired checkpoints --
      //
      //      std::vector<int> cp_coords // checkpoint coordinates are determined by:
      //                                    cp_coords.push_back(cp_begin_y);
      //                                    int cp_total_d = max(cp_begin_y, cp_end_y) - min(cp_begin_y, cp_end_y);
      //                                    for(i = 0; i < checkpoint_quantity; i++){
      //                                      int cp_coord = cp_begin_y +/- (cp_total_d/cp_quantity) * (i+1);
      //                                      cp_coords.push_back(cp_coord);
      //                                    }
      //                                    cp_coords.push_back(cp_end_y);
      //
      // On each frame, compare elements (uniqueID) in "IDs_To_Track" to "vehicles[i].detection_ID"
      //      if match: check if vehicles[i].y.isMember(cp_coords)
      //        if true: determine the "checkpoint_number" based on vehicles[i].y (as ROIs are placed evenly apart)
      //                 if checkpoint_number == cp_number_end:
      //                      i. the vehicle is at the end of the ROI. using accumulated "checkpoint_number" and "timestamps", calculate vehicle velocity
      //                         velocity = cp_distance.inMeters / (timestamp.last() - timestamp.first());
      //                      ii. append to msg_vehicles,
      //
      //                          e.g. msg_vehicle vehicle_to_push = {unique_ID, velocity, Class, checkpoint_numbers};
      //                              msg_vehicles.push_back(vehicle_to_push);
      //
      //                 else, the vehicle has not yet reached the end of the checkpoint, therefore:
      //                   i. from vector "tracked_vehicles" find the index number of the element with uniqueID == vehicles[i].detection_ID
      //
      //                      e.g. int index_number;
      //                           for(int j = 0; j < tracked_vehicles.size(); j++){
      //                            if(tracked_vehicles[j].uniqueID == vehicles[i].detection_ID){
      //                             index_number = j;
      //                            }
      //                           }
      //
      //                   ii. once index number is found, access the instance of "tracked_vehicle" with tracked_vehicles[index_number].
      //                        append the checkpoint number and timestamp.
      //      if no match: do nothing
      //

      /*** BEGIN DEBUG MESSAGE ***/
      #ifdef ENABLE_DEBUG_MODE
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "----------\n";
        export_csv << "Finished tracking\n";
        export_csv.close();
      #endif
      /*** END DEBUG MESSAGE ***/

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

      /*** BEGIN DEBUG MESSAGE ***/
      #ifdef ENABLE_DEBUG_MODE
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "Listing detected vehicles in frame: " << frame_count << "\n";
        export_csv.close();
        for(int i = 0; i < vehicles.size(); i++){
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "UniqueID:" << vehicles[i].detection_ID <<"\n";
          export_csv << "X: " << vehicles[i].x <<"\n";
          export_csv << "Y: " << vehicles[i].y <<"\n";
          export_csv.close();
          ROS_INFO("Unique ID: %d\n", vehicles[i].detection_ID);
          ROS_INFO("X: %d\n", vehicles[i].x);
          ROS_INFO("Y: %d\n", vehicles[i].y);
        }
      #endif
      /*** END DEBUG MESSAGE ***/

      if(vehicles.size() != 0){
        /*** BEGIN DEBUG MESSAGE ***/
        #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "----------\n";
          export_csv << "Overwriting current frame vehicles to previous vehicles\n";
          export_csv.close();
        #endif
        /*** END DEBUG MESSAGE ***/
        prev_vehicles = vehicles; // only overwrite when new vehicles infos are present
      }

    }

    /*** BEGIN DEBUG MESSAGE ***/
    #ifdef ENABLE_DEBUG_MODE
      export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
      export_csv << "----------\n";
      export_csv << "Wiping current frame vehicles\n";
      export_csv.close();
    #endif
    /*** END DEBUG MESSAGE ***/
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
        vehicle current = {i, vehicle_class, x_center, y_center, x_dimension_bbox, y_dimension_bbox};
        vehicles.push_back(current);
    }
  }

}
//
void extract_count(const std_msgs::Int8::ConstPtr& count_value){

    bbox_no = count_value->data;

}
