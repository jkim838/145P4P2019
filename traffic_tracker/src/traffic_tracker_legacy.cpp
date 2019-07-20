#include "traffic_tracker.hpp"

int main(int arg, char **argv){

  ros::init(arg, argv, "traffic_tracker");
  ros::NodeHandle tt_nh;
  ros::Rate loop_rate(1000);

  initialize_vri();
  #ifdef ROI_DEBUG_MODE
    std::ofstream ROI_csv;
    ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
    ROI_csv << "Initializing ROIS:\n";
    ROI_csv << "cpID:";
    for(int i = 0; i < cp_coords_y.size(); i++){
      ROI_csv << i <<",";
    }
    ROI_csv << "\ny-coord:";
    for(int i = 0; i < cp_coords_y.size(); i++){
      ROI_csv << cp_coords_y[i] <<",";
    }
    ROI_csv << "\n";
    ROI_csv << "==========\n";
    ROI_csv.close();
  #endif

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
        std::string current_class = vehicles[i].vehicleClass;
        long int current_ID = vehicles[i].detectionID;
        long int current_xDimension = vehicles[i].xDimension;
        long int current_yDimension = vehicles[i].yDimension;
        prev_vehicles[i] = {current_ID, current_class, current_x, current_y, current_xDimension, current_yDimension};
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
          std::string current_class = vehicles[i].vehicleClass;
          long int current_xDimension = vehicles[i].xDimension;
          long int current_yDimension = vehicles[i].yDimension;
          vehicles[i] = {i, current_class, current_x, current_y, current_xDimension, current_yDimension};
        }

        /***BEGIN DEBUG MESSAGE ***/
        #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "vehicle is " << vehicles.size() << " wide\n";
          export_csv << "list of elements: \n";
          export_csv << "ID:";
          for(size_t i = 0; i < vehicles.size(); i++){
            export_csv << vehicles[i].detectionID << ",";
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
            export_csv << prev_vehicles[i].detectionID << ",";
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
        std::string current_class = vehicles[i].vehicleClass; //hold on to current class
        long int current_xDimension = vehicles[i].xDimension;
        long int current_yDimension = vehicles[i].yDimension;

        long int current_x_min = current_x - (current_xDimension / 2); //minimum x-coordinate value, x1
        long int current_x_max = current_x + (current_xDimension / 2); //maximum x-coordinate value, x2
        long int current_y_min = current_y - (current_yDimension / 2); //minimum y-coordinate value, y1
        long int current_y_max = current_y + (current_yDimension / 2); //maximum y-coordinate value, y2
        long int area_current = (current_x_max - current_x_min) * (current_y_max - current_y_min); // calculate the area of bounding box in px...

        int iteration_count = 0;

        // BUG: This wouldn't run if there's no data in prev_vehicle
        for(size_t j = 0; j < prev_vehicles.size(); j++){

          long int prev_x = prev_vehicles[j].x;
          long int prev_y = prev_vehicles[j].y;
          long int prev_xDimension = prev_vehicles[j].xDimension;
          long int prev_yDimension = prev_vehicles[j].yDimension;

          long int previous_x_min = prev_x - (prev_xDimension / 2); // minimum x-coordinate value, x1
          long int previous_x_max = prev_x + (prev_xDimension / 2); // maximum x-coordinate value, x2
          long int previous_y_min = prev_y - (prev_yDimension / 2); // minimum y-coordinate value, y1
          long int previous_y_max = prev_y + (prev_yDimension / 2); // maximum y-coordinate value, y2

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

          float IOU_Threshold = 0.20;
          int abs_x_diff = abs(vehicles[i].x - prev_vehicles[j].x);
          int abs_y_diff = abs(vehicles[i].y - prev_vehicles[j].y);
          int euclid_distance = hypot(abs_x_diff, abs_y_diff);
          int euclid_threshold = hypot(current_xDimension, current_yDimension)/6; // give strict Euclidean distance threshold as it is only a back-up threshold

          /*** BEGIN DEBUG MESSAGE ***/
          #ifdef ENABLE_DEBUG_MODE
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "Comparing element "<< vehicles[i].detectionID << " in current frame to element " << prev_vehicles[j].detectionID << " in the previous frame\n";
            export_csv << "Current element: " << vehicles[i].detectionID << "\n";
            export_csv << "Center X: " << current_x << "\n";
            export_csv << "Center Y: " << current_y << "\n";
            export_csv << "X-Dimension: " << current_xDimension << "\n";
            export_csv << "Y-Dimension: " << current_yDimension << "\n";
            export_csv << "Area: " << current_yDimension * current_xDimension << "\n";
            export_csv << "----------\n";
            export_csv << "Previous element: " << prev_vehicles[j].detectionID << "\n";
            export_csv << "Center X: " << prev_x << "\n";
            export_csv << "Center Y: " << prev_y << "\n";
            export_csv << "X-Dimension: " << prev_xDimension << "\n";
            export_csv << "Y-Dimension: " << prev_yDimension << "\n";
            export_csv << "Area: " << prev_yDimension * prev_xDimension << "\n";
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
              export_csv << "Match between " << vehicles[i].detectionID << " and " << prev_vehicles[j].detectionID <<"\n";
              export_csv << "Reassigning Unique ID:" << prev_vehicles[j].detectionID << " to current element, " << vehicles[i].detectionID <<"\n";
              export_csv << "----------\n";
              export_csv.close();
            #endif
            /*** END DEBUG MESSAGE ***/

            vehicles[i] = {prev_vehicles[j].detectionID, prev_vehicles[j].vehicleClass, current_x, current_y, current_xDimension, current_yDimension};
            //give current element unique ID and class of the matching previous frame
            match_found = true; // match was found for this element in previous frame (i.e. not a new vehicle)

          }
          iteration_count++;

        }

        //Need to find the maximum ID EVER assigned in history of prev_vehicles...
        for(int i = 0; i < vehicles.size(); i++){
          if(maximum_ID < vehicles[i].detectionID){
            maximum_ID = vehicles[i].detectionID;
          }
        }

        if(!match_found && prev_vehicles.size() != 0 && iteration_count == prev_vehicles.size()){
          // if no match for current element is found in previous frame, this is a new vehicle entering the scene,

          /*** BEGIN DEBUG MESSAGE ***/
          #ifdef ENABLE_DEBUG_MODE
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "No match found for the element: " << vehicles[i].detectionID <<"\n";
            export_csv.close();
          #endif
          /*** END DEBUG MESSAGE***/

          int newUniqueID = maximum_ID + 1;

          /*** BEGIN DEBUG MESSAGE ***/
          #ifdef ENABLE_DEBUG_MODE
            export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
            export_csv << "Global maximum unique ID was: " << maximum_ID << "\n";
            export_csv << "Assigning a new Unique ID:" << newUniqueID << " to current element, " << vehicles[i].detectionID <<"\n";
            export_csv.close();
          #endif
          /*** END DEBUG MESSAGE ***/

          vehicles[i] = {newUniqueID, current_class, current_x, current_y, current_xDimension, current_yDimension};
        }

      }

      /*** DEBUG: VRI CODE FROM HERE ***/

      #ifdef ROI_DEBUG_MODE
        std::ofstream ROI_csv;
        ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
        ROI_csv << "==========\n";
        ROI_csv << "Frame ID:" << frame_count <<"\n";
        ROI_csv << "==========\n";
        ROI_csv.close();
      #endif

      /*** ITERATE THROUGH ALL VEHICLES IN THE FRAME ***/
      for(int i = 0; i < vehicles.size(); i++){
        /*** THE VEHICLE IS ENTERING THE FIRST ROI ***/
        if(vehicles[i].y > (cp_begin_y - 30) && vehicles[i].y <= (cp_begin_y + 30)){
          // See if there is a previous entry with same unique ID. If no, create a new entry to TrackingVehicles
          bool id_match = false;
          int id_to_find = vehicles[i].detectionID;
          for(int j = 0; j < TrackingVehicles.size(); j++){
            if(TrackingVehicles[j].uniqueID == id_to_find){
              // a match has been found. previous entry exists
              id_match = true;
              // see cpID was already recorded for this element
              bool already_appended = false;
              for(int k = 0; k < TrackingVehicles[j].checkpoints.size(); k++){
                if(TrackingVehicles[j].checkpoints[k] == 0){
                  already_appended = true;
                }
              }
              if(!already_appended){
                // checkpoint ID (cpID) and timestamp is not yet recorded. Do so now.
                unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();;
                TrackingVehicles[j].timestamps.push_back(timestamp_now);
                TrackingVehicles[j].checkpoints.push_back(0);
                #ifdef ROI_DEBUG_MODE
                  ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
                  ROI_csv << "Element ID:" << vehicles[i].detectionID <<" entered ROI:" << 0 << "\n";
                  ROI_csv.close();
                #endif
              }
            }
          }

          if(!id_match){
            // no match found... first of its kind...
            // Get current timestamp
            unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();; //dummy value
            std::vector<unsigned long> timestamp_vector;
            timestamp_vector.push_back(timestamp_now);

            std::vector<int> cpID_vector;
            cpID_vector.push_back(0);

            // Create an instance of tracked_vehicle for currently observed vehicle
            tracked_vehicle currentVehicle = {vehicles[i].detectionID, vehicles[i].vehicleClass, timestamp_vector,cpID_vector, frame_count};
            TrackingVehicles.push_back(currentVehicle);

            #ifdef ROI_DEBUG_MODE
              ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
              ROI_csv << "Tracking initialized for ID:" << vehicles[i].detectionID <<" at ROI:" << 0 << "\n";
              ROI_csv.close();
            #endif
          }
        }

        /***TODO: ADD A CASE WHERE VEHICLE ENTERS n-th ROI ***/
        else if(vehicles[i].y >= (655 - 30) && vehicles[i].y <= (655 + 30)){
          bool id_match = false;
          int id_to_find = vehicles[i].detectionID;
          for(int j = 0; j < TrackingVehicles.size(); j++){
            if(TrackingVehicles[j].uniqueID == id_to_find){
              // a match has been found. previous entry exists
              id_match = true;
              // see cpID was already recorded for this element
              bool already_appended = false;
              for(int k = 0; k < TrackingVehicles[j].checkpoints.size(); k++){
                if(TrackingVehicles[j].checkpoints[k] == 1){
                  already_appended = true;
                }
              }
              if(!already_appended){
                // checkpoint ID (cpID) and timestamp is not yet recorded. Do so now.
                unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();; //dummy value
                TrackingVehicles[j].timestamps.push_back(timestamp_now);
                TrackingVehicles[j].checkpoints.push_back(1);
                #ifdef ROI_DEBUG_MODE
                  ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
                  ROI_csv << "Element ID:" << vehicles[i].detectionID <<" entered ROI:" << 1 << "\n";
                  ROI_csv.close();
                #endif
              }
            }
          }

          if(!id_match){
            // no match found... first of its kind...
            // Get current timestamp
            unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();; //dummy value
            std::vector<unsigned long> timestamp_vector;
            timestamp_vector.push_back(timestamp_now);

            std::vector<int> cpID_vector;
            cpID_vector.push_back(1);

            // Create an instance of tracked_vehicle for currently observed vehicle
            tracked_vehicle currentVehicle = {vehicles[i].detectionID, vehicles[i].vehicleClass, timestamp_vector,cpID_vector, frame_count};
            TrackingVehicles.push_back(currentVehicle);

            #ifdef ROI_DEBUG_MODE
              ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
              ROI_csv << "Tracking initialized for ID:" << vehicles[i].detectionID <<" at ROI:" << 1 << "\n";
              ROI_csv.close();
            #endif
          }
        }

        else if(vehicles[i].y >= (430 - 30) && vehicles[i].y <= (430 + 30)){
          bool id_match = false;
          int id_to_find = vehicles[i].detectionID;
          for(int j = 0; j < TrackingVehicles.size(); j++){
            if(TrackingVehicles[j].uniqueID == id_to_find){
              // a match has been found. previous entry exists
              id_match = true;
              // see cpID was already recorded for this element
              bool already_appended = false;
              for(int k = 0; k < TrackingVehicles[j].checkpoints.size(); k++){
                if(TrackingVehicles[j].checkpoints[k] == 2){
                  already_appended = true;
                }
              }
              if(!already_appended){
                // checkpoint ID (cpID) and timestamp is not yet recorded. Do so now.
                unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();; //dummy value
                TrackingVehicles[j].timestamps.push_back(timestamp_now);
                TrackingVehicles[j].checkpoints.push_back(2);
                #ifdef ROI_DEBUG_MODE
                  ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
                  ROI_csv << "Element ID:" << vehicles[i].detectionID <<" entered ROI:" << 2 << "\n";
                  ROI_csv.close();
                #endif
              }
            }
          }

          if(!id_match){
            // no match found... first of its kind...
            // Get current timestamp
            unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();; //dummy value
            std::vector<unsigned long> timestamp_vector;
            timestamp_vector.push_back(timestamp_now);

            std::vector<int> cpID_vector;
            cpID_vector.push_back(2);

            // Create an instance of tracked_vehicle for currently observed vehicle
            tracked_vehicle currentVehicle = {vehicles[i].detectionID, vehicles[i].vehicleClass, timestamp_vector,cpID_vector, frame_count};
            TrackingVehicles.push_back(currentVehicle);

            #ifdef ROI_DEBUG_MODE
              ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
              ROI_csv << "Tracking initialized for ID:" << vehicles[i].detectionID <<" at ROI:" << 2 << "\n";
              ROI_csv.close();
            #endif
          }
        }

        /*** VEHICLE HAS LEFT THE LAST ROI ***/
        else if(vehicles[i].y >= (cp_end_y - 30) && vehicles[i].y <= (cp_end_y + 30)){

          // DEBUG: Find the cpID of the last ROI -- verified code as working at 1417
          #ifdef ROI_DEBUG_MODE
            int cpID;
            for(int j = 0; j < cp_coords_y.size(); j++){
              if(vehicles[i].y > (cp_coords_y[j] - 5) && vehicles[i].y < (cp_coords_y[j] + 5)){
                cpID = j;
              }
            }
          #endif

          /*** Appending Last ROI Checkpoint and Timestamp ***/
          // Search through TrackingVehicles to find matching element. Append timestamp and CPID to the element.
          int id_to_find = vehicles[i].detectionID;
          for(int j = 0; j < TrackingVehicles.size(); j++){
            if(TrackingVehicles[j].uniqueID == id_to_find){
              // a match has been found.
              // see cpID was already recorded for this element
              bool already_appended = false;
              for(int k = 0; k < TrackingVehicles[j].checkpoints.size(); k++){
                if(TrackingVehicles[j].checkpoints[k] == 3){
                  already_appended = true;
                }
              }
              if(!already_appended){
                unsigned long timestamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();; //dummy value
                TrackingVehicles[j].timestamps.push_back(timestamp_now);
                TrackingVehicles[j].checkpoints.push_back(3);
                #ifdef ROI_DEBUG_MODE
                  ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
                  ROI_csv << "Tracking ended for ID:" << vehicles[i].detectionID <<" at ROI:" << 3 << "\n";
                  ROI_csv << "==========:\n";
                  ROI_csv.close();
                #endif
              }
            }
          }

          #ifdef ROI_DEBUG_MODE
            ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
            ROI_csv << "TrackingVehicles Elements:\n";
            for(int i = 0; i < TrackingVehicles.size(); i++){
              ROI_csv << "{ID:" << TrackingVehicles[i].uniqueID << ", Class:" << TrackingVehicles[i].vehicleClass << ", Timestamps:[";
              for(int j = 0; j < TrackingVehicles[i].timestamps.size(); j++){
                ROI_csv << TrackingVehicles[i].timestamps[j] << ",";
              }
              ROI_csv << "], Checkpoints:[";
              for(int j = 0; j < TrackingVehicles[i].checkpoints.size(); j++){
                ROI_csv << TrackingVehicles[i].checkpoints[j] << ",";
              }
              ROI_csv << "], FOI:" << TrackingVehicles[i].frameInit << "}\n";
            }
            ROI_csv.close();
          #endif

          /***TODO : PUBLISH MESSAGE HERE***/

          /*** ERASING ELEMENTS FROM TRACKINGVEHICLES ***/
          //find from the TrackingVehicles using UniqueID
          for(int j = 0; j < TrackingVehicles.size(); j++){
            if(TrackingVehicles[j].uniqueID == vehicles[i].detectionID){
              TrackingVehicles.erase(TrackingVehicles.begin() + j);
              #ifdef ROI_DEBUG_MODE
                ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
                ROI_csv << "==========:\n";
                ROI_csv << "Erasing element ID:" << vehicles[i].detectionID <<" from the TrackingVehicles\n";
                ROI_csv << "==========:\n";
                ROI_csv.close();
              #endif
            }
          }
          //TODO: IMPLEMENT A FEATURE WHERE ELEMENTS THAT ARE TOO OLD ARE ERASED
        }
      }

      #ifdef ROI_DEBUG_MODE
        ROI_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
        ROI_csv << "==========:\n";
        ROI_csv << "Tracked Vehicles Final to Frame:\n";
        for(int i = 0; i < TrackingVehicles.size(); i++){
          ROI_csv << "{ID:" << TrackingVehicles[i].uniqueID << ", Class:" << TrackingVehicles[i].vehicleClass << ", Timestamps:[";
          for(int j = 0; j < TrackingVehicles[i].timestamps.size(); j++){
            ROI_csv << TrackingVehicles[i].timestamps[j] << ",";
          }
          ROI_csv << "], Checkpoints:[";
          for(int j = 0; j < TrackingVehicles[i].checkpoints.size(); j++){
            ROI_csv << TrackingVehicles[i].checkpoints[j] << ",";
          }
          ROI_csv << "], FOI:" << TrackingVehicles[i].frameInit << "}\n";
        }
        ROI_csv.close();
      #endif


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
        toString << vehicles[i].detectionID;
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
          export_csv << "UniqueID:" << vehicles[i].detectionID <<"\n";
          export_csv << "X: " << vehicles[i].x <<"\n";
          export_csv << "Y: " << vehicles[i].y <<"\n";
          export_csv.close();
          ROS_INFO("Unique ID: %d\n", vehicles[i].detectionID);
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
    std::string vehicleClass = bbox->bounding_boxes[i].Class;
    long int xDimension_bbox = (max_x - min_x);
    long int yDimension_bbox = (max_y - min_y);
    long int x_center = min_x + xDimension_bbox/2;
    long int y_center = min_y + yDimension_bbox/2;
    if(vehicleClass=="bicycle"||vehicleClass=="car"||vehicleClass=="motorbike"||vehicleClass=="bus"||vehicleClass=="truck"){
        vehicle current = {i, vehicleClass, x_center, y_center, xDimension_bbox, yDimension_bbox};
        vehicles.push_back(current);
    }
  }

}
//
void extract_count(const std_msgs::Int8::ConstPtr& count_value){

    bbox_no = count_value->data;

}

