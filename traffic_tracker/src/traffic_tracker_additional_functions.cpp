#include "traffic_tracker.hpp"

void getBBOXinfo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox)
{
  long int vehicleID = 0;
  for(auto it = bbox->bounding_boxes.begin();
      it != bbox->bounding_boxes.end(); ++it)
  {
    long int minX = (*it).xmin;
    long int minY = (*it).ymin;
    long int maxX = (*it).xmax;
    long int maxY = (*it).ymax;
    std::string vehicleClass = (*it).Class;
    long int xDimension = (maxX - minX);
    long int yDimension = (maxY - minY);
    long int xCenter = minX + xDimension/2;
    long int yCenter = minY + yDimension/2;
    if(vehicleClass=="car"||vehicleClass=="motorbike"||vehicleClass=="bus"||
       vehicleClass=="truck")
    {
      vehicle current =
      {
        vehicleID,
        vehicleClass,
        xCenter,
        yCenter,
        xDimension,
        yDimension
      };
      vehicles.push_back(current);
      vehicleID++;
    }
  }
}

void getFrameFromSource(const sensor_msgs::Image::ConstPtr& detection_image)
{
  try
  {
    cv_bridge::CvImageConstPtr take_cv;
    take_cv = cv_bridge::toCvCopy(detection_image);
    frame = take_cv->image;
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Error: cannot convert message to image",
    detection_image->encoding.c_str());
  }
}

void displayFeed(std::string windowName, cv::Mat imageName)
{
  #ifdef ENABLE_DEBUG_MODE
  cv::circle(frame, cv::Point(716,270), 10, cv::Scalar(0,255,0), 2, 1);
  cv::circle(frame, cv::Point(1170,270), 10, cv::Scalar(0,255,0), 2, 1);
  cv::circle(frame, cv::Point(310,860), 10, cv::Scalar(0,255,0), 2, 1);
  cv::circle(frame, cv::Point(1530,860), 10, cv::Scalar(0,255,0), 2, 1);
  #endif
  cv::imshow(windowName, imageName);
  cv::waitKey(30);
}

void preprocessVehicles()
{
  bool warnVehicleEmpty = false;
  if(vehicles.size() == 0)
  {
    warnVehicleEmpty = true;
  }
  else
  {
    int preprocessVehicleID = 0;
    for(auto it = vehicles.begin();it!= vehicles.end(); ++it){
      vehicle toPaste =
      {
        preprocessVehicleID,
        (*it).vehicleClass,
        (*it).x,
        (*it).y,
        (*it).xDimension,
        (*it).yDimension
      };
      if(frame_count == 1)
      {
        prev_vehicles.push_back(toPaste);
      }
      else
      {
        *it = toPaste;
      }
      preprocessVehicleID++;
    }
  }

  #ifdef ENABLE_DEBUG_MODE
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
    if(warnVehicleEmpty)
    {
      export_csv << "WARNING: No data in vector: vehicles\n";
    }
    else
    {
      export_csv << """vehicle"" is:" << vehicles.size() << " elements wide\n";
      export_csv << """prev_vehicle"" is:" << prev_vehicles.size()
                 << " elements wide\n";
      export_csv << "Vehicle in current frame:\n";
      for(auto it = vehicles.begin();
      it!= vehicles.end();++it)
      {
        export_csv
        << "{ID:" << (*it).detectionID
        << ", Class:" << (*it).vehicleClass
        << ", X-center:" << (*it).x
        << ", Y-center:" << (*it).y
        << ", X-length:" << (*it).xDimension
        << ", Y-length:" << (*it).yDimension
        << "}\n";
      }
    }
    export_csv << "\n";
    export_csv.close();
  #endif
}

void beginTracking()
{
  for(auto currentFrameIt = vehicles.begin();
      currentFrameIt != vehicles.end(); ++currentFrameIt)
  {
    bool MatchFound = false;
    long int currentX = (*currentFrameIt).x;
    long int currentY = (*currentFrameIt).y;
    long int currentXDimension = (*currentFrameIt).xDimension;
    long int currentYDimension = (*currentFrameIt).yDimension;
    long int currentXMin = currentX - (currentXDimension / 2);
    long int currentXMax = currentX + (currentXDimension / 2);
    long int currentYMin = currentY - (currentYDimension / 2);
    long int currentYMax = currentY + (currentYDimension / 2);
    long int areaCurrent = currentXDimension * currentYDimension;
    std::string currentClass = (*currentFrameIt).vehicleClass;

    if(maximum_ID < (*currentFrameIt).detectionID){
      maximum_ID = (*currentFrameIt).detectionID;
    }

    for(auto previousFrameIt = prev_vehicles.begin();
        previousFrameIt != prev_vehicles.end(); ++previousFrameIt)
    {
      long int prevX = (*previousFrameIt).x;
      long int prevY = (*previousFrameIt).y;
      long int prevXDimension = (*previousFrameIt).xDimension;
      long int prevYDimension = (*previousFrameIt).yDimension;

      long int prevXMin = prevX - (prevXDimension / 2); // minimum x-coordinate value, x1
      long int prevXMax = prevX + (prevXDimension / 2); // maximum x-coordinate value, x2
      long int prevYMin = prevY - (prevYDimension / 2); // minimum y-coordinate value, y1
      long int prevYMax = prevY + (prevYDimension / 2); // maximum y-coordinate value, y2

      long int interX1 = std::max(currentXMin, prevXMin); // minimum x-coordinate value of intersecting rectangle
      long int interX2 = std::min(currentXMax, prevXMax); // maximum x-coordinate value of the intersecting rectangle
      long int interY1 = std::max(currentYMin, prevYMin); // minimum y-coordinate of the intersecting rectangle
      long int interY2 = std::min(currentYMax, prevYMax); // maximum y-coordinate of the intersecting rectangle
      long int interWidth = (interX2 - interX1);
      long int interHeight = (interY2 - interY1);

      long int areaInter;
      if(interWidth < 0 || interHeight < 0){
        areaInter = 0; // for a case where two bounding boxes do not overlap...
      }
      else{
        areaInter = interWidth * interHeight;
      }

      long int areaPrev = (prevYMax - prevYMin) * (prevXMax - prevXMin);
      long int areaUnion = areaCurrent + areaPrev - areaInter;

      float IOU = (float)areaInter / (float)areaUnion;

      float IOU_Threshold = 0.20;
      int absXDiff = abs((*currentFrameIt).x - (*previousFrameIt).x);
      int absYDiff = abs((*currentFrameIt).y - (*previousFrameIt).y);
      int euclidDistance = hypot(absXDiff, absYDiff);
      int euclidThreshold = hypot(currentXDimension, currentYDimension)/6; // give strict Euclidean distance threshold as it is only a back-up threshold

      #ifdef ENABLE_DEBUG_MODE
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "===================\n";
        export_csv << "Comparing elements:\n";
        export_csv << "Current frame element:"
                   << "{ID:" << (*currentFrameIt).detectionID
                   << ", Center X:" << currentX
                   << ", Center Y:" << currentY
                   << ", X-Length:" << currentXDimension
                   << ", Y-Length:" << currentYDimension
                   << ", Area:" << areaCurrent
                   << "}\n";
        export_csv << "Previous frame element:"
                   << "{ID:" << (*previousFrameIt).detectionID
                   << ", Center X:" << prevX
                   << ", Center Y:" << prevY
                   << ", X-Length:" << prevXDimension
                   << ", Y-Length:" << prevYDimension
                   << ", Area:" << areaPrev
                   << "}\n";
        export_csv << "Area of Intersection:" << areaInter <<"\n";
        export_csv << "Area of Union:" << areaUnion <<"\n";
        export_csv << "IOU:" << IOU <<"\n";
        export_csv << "Euclidean Distance:" << euclidDistance << "\n";
        export_csv << "IOU Threshold: "<< IOU_Threshold << "\n";
        export_csv << "Euclid Threshold: "<< euclidThreshold << "\n";
        export_csv.close();
      #endif

      if(IOU > IOU_Threshold || euclidDistance < euclidThreshold)
      {
        #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
          export_csv << "----------\n";
          export_csv << "Match between " << (*currentFrameIt).detectionID << ", "
                     << (*previousFrameIt).detectionID <<"\n";
          export_csv << "Reassigning Unique ID:" << (*previousFrameIt).detectionID
                     << " ->current frame element ID:" << (*currentFrameIt).detectionID <<"\n";
          export_csv.close();
        #endif
        vehicle toPaste =
        {
          (*previousFrameIt).detectionID,
          (*previousFrameIt).vehicleClass,
          currentX,
          currentY,
          currentXDimension,
          currentYDimension
        };
        *currentFrameIt = toPaste;
        MatchFound = true;
      }
    }
    if(!MatchFound && prev_vehicles.size() != 0)
    {
      int newUniqueID = maximum_ID + 1;
      maximum_ID = newUniqueID;
      vehicle toPaste =
      {
        newUniqueID,
        currentClass,
        currentX,
        currentY,
        currentXDimension,
        currentYDimension
      };
      *currentFrameIt = toPaste;

      #ifdef ENABLE_DEBUG_MODE
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
        export_csv << "No match found for the element ID:"
                   << (*currentFrameIt).detectionID <<"\n";
        export_csv << "Global maximum unique ID:" << maximum_ID << "\n";
        export_csv << "Assigning a new Unique ID:" << newUniqueID
                   << "->current element ID:" << (*currentFrameIt).detectionID <<"\n";
        export_csv.close();
      #endif
    }

    //IDEA: COULD WE USE THIS CENTERPOINT (OPENCV) TO USE AS VEHICLE LOCATION AFTER BEING PERSPECTIVE WARPED?
    cv::Point centerPoint = cv::Point((*currentFrameIt).x, (*currentFrameIt).y);


    // redefine variables in a suitable form for perspectiveTransform function...
    // cv::perspectiveTransform(input array, output array, input matrix)
    ppCenterPointIn.push_back(centerPoint);
    // apply perspective transform...
    cv::perspectiveTransform(ppCenterPointIn, ppCenterPointOut, ppMatrix);
    // make transformed coordinates and ID global across the program
    std::vector<cv::Point2f> toPasteCoord;
    std::vector<long int> toPasteFrame;
    toPasteCoord.push_back(ppCenterPointOut[0]);
    toPasteFrame.push_back(frame_count);
    ppVehicleFrame.push_back({(*currentFrameIt).detectionID,
                              (*currentFrameIt).vehicleClass,
                              toPasteCoord,
                              toPasteFrame});
    toPasteCoord.clear();
    toPasteFrame.clear();
    #ifdef ENABLE_PERSPECTIVE_FEED
    #ifdef DRAW_PERSPECTIVE_INFO
    // Plot green-circle and print uniqueID of the vehicle...
    cv::circle(ppImage, ppCenterPointOut[0], 10, cv::Scalar(0,255,0), 2, 1);
    std::stringstream toPPString;
    toPPString << (*currentFrameIt).detectionID;
    cv::putText(ppImage, toPPString.str(), ppCenterPointOut[0],
                cv::FONT_HERSHEY_SIMPLEX,
                0.75, cv::Scalar(0,0,255),2);
    // clear ppCenterPointIn so it only has one element only
    ppCenterPointIn.clear();
    #endif
    #endif

    // if perspectiveTransform is disabled, plot blue circle on a normal feed
    #ifdef ENABLE_TRACKER_FEED
    #ifdef DRAW_TRAKCER_INFO
    cv::circle(frame, centerPoint, 10, cv::Scalar(0,255,0), 2, 1);
    std::stringstream toString;
    toString << (*currentFrameIt).detectionID;
    cv::putText(frame, "Class:"+(*currentFrameIt).vehicleClass+" ID:"
                +toString.str(), centerPoint, cv::FONT_HERSHEY_SIMPLEX,
                1.25, cv::Scalar(0,0,255),2);
    #endif
    #endif
  }

  #ifdef ENABLE_DEBUG_MODE
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/perspective_debugging.csv", std::ofstream::app);
  export_csv << "==========\n";
  export_csv << "Frame ID:" << frame_count <<"\n";
  export_csv << "==========\n";
  export_csv << "List of vehicles in the frame:\n";
  for(auto ppIt = ppVehicleFrame.begin(); ppIt != ppVehicleFrame.end(); ++ppIt)
  {
    export_csv << "{UniqueID:" << (*ppIt).uniqueID
               << ", Class:" << (*ppIt).vehicleClass
               << ", Coordinate:(" << (*ppIt).centerPoint.back().x << ", "
               << (*ppIt).centerPoint.back().y << "), Frame:"
               << (*ppIt).frameNo.back()
               << "}\n";
  }
  export_csv.close();
  #endif
}

void extractPerspectiveCoord()
{
    // to be implemented
    #ifdef ENABLE_DEBUG_MODE
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
    export_csv << "==========\n";
    export_csv << "Frame ID:" << frame_count <<"\n";
    export_csv << "==========\n";
    export_csv << "List of vehicles in the frame:\n";
    export_csv.close();
    #endif

    for(auto ppIt = ppVehicleFrame.begin(); ppIt != ppVehicleFrame.end(); ++ppIt)
    {
      // if the vehicle enters the zone
      if((*ppIt).centerPoint.back().y <= 1020 && (*ppIt).centerPoint.back().y >= 195)
      {
        // check if there is a previous entry to this vehicle
        // if there is no entry, make one.
        // if there is an entry, append information to specific entry.
        bool idMatch = false;
        for(auto vIt = TrackedVehicles.begin(); vIt != TrackedVehicles.end(); ++vIt)
        {
          if((*vIt).uniqueID == (*ppIt).uniqueID)
          {
            // ID Match is found... the previous entry exists.
            idMatch = true;
            // only append coordinates at every 10th frame..
            if(frame_count % ((*vIt).frameNo.front() + 10) == 0)
            {
              // update TrackedVehicles
              (*vIt).centerPoint.push_back((*ppIt).centerPoint.back());
              (*vIt).frameNo.push_back((*ppIt).frameNo.back());

              #ifdef ENABLE_DEBUG_MODE
              export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
              export_csv << "==========\n";
              export_csv << "Appending new coordinates for ID:"
                         << (*ppIt).uniqueID << "at frame:" << frame_count << "\n";
              export_csv.close();
              #endif
            }
          }
        }
        // no idMatch was found after searching through all vehicles...
        if(!idMatch)
        {
          TrackedVehicles.push_back((*ppIt));
          #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
          export_csv << "Tracking initialized for ID:"
                     << (*ppIt).uniqueID << " at frame:" << frame_count << "\n";
          export_csv.close();
          #endif
        }
      }
      else if((*ppIt).centerPoint.back().y < 195)
      {
        // vehicle left the zone
        #ifdef ENABLE_DEBUG_MODE
        export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
        export_csv << "Tracking ended for ID:"
                   << (*ppIt).uniqueID << " at frame:" << frame_count << "\n";
        export_csv.close();
        #endif

        bool idMatch = false;
        for(auto eVIt = EndTrackVehicles.begin(); eVIt != EndTrackVehicles.end(); ++eVIt)
        {
          if((*eVIt) == (*ppIt).uniqueID)
          {
            // do nothing when there is a match
            idMatch = true;
          }
        }
        if(!idMatch)
        {
          EndTrackVehicles.push_back((*ppIt).uniqueID);
        }
      }
    }

    #ifdef ENABLE_DEBUG_MODE
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
    export_csv << "==========\n";
    export_csv << "List of entries to delete at the end of the frame:\n";
    for(auto eVIt = EndTrackVehicles.begin(); eVIt != EndTrackVehicles.end(); ++eVIt)
    {
      export_csv << "{ID:" << (*eVIt) << "},";
    }
    export_csv << "\n";
    export_csv.close();
    #endif

    // DEBUG: publish message
    for(auto vIt = TrackedVehicles.begin(); vIt != TrackedVehicles.end(); ++vIt)
    {
      traffic_tracker::perspectiveVehicle toPaste;
      toPaste.uniqueID = (*vIt).uniqueID;
      toPaste.vehicleClass = (*vIt).vehicleClass;
      traffic_tracker::point2f toPasteCoord;
      for(auto vcIt = (*vIt).centerPoint.begin();
          vcIt != (*vIt).centerPoint.end(); ++vcIt)
      {
        toPasteCoord.x = (*vcIt).x;
        toPasteCoord.y = (*vcIt).y;
        toPaste.centerPoint.push_back(toPasteCoord);
      }
      for(auto vfIt = (*vIt).frameNo.begin(); vfIt != (*vIt).frameNo.end(); ++vfIt)
      {
        toPaste.frameNo.push_back((*vfIt));
      }
      msg.trackerOutput.push_back(toPaste);
    }
    tt_tracker_pub.publish(msg);
    msg.trackerOutput.clear();

    #ifdef ENABLE_DEBUG_MODE
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
    export_csv << "==========\n";
    export_csv << "Attempting to publish message...\n";
    export_csv.close();
    #endif
    // find the vehicle that left the zone and erase from list of tracking
    for(auto eVIt = EndTrackVehicles.begin(); eVIt != EndTrackVehicles.end(); ++eVIt)
    {
      for(auto vIt = TrackedVehicles.begin(); vIt != TrackedVehicles.end();)
      {
        if((*vIt).uniqueID == (*eVIt))
        {

          #ifdef ENABLE_DEBUG_MODE
          export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
          export_csv << "==========\n";
          export_csv << "Erasing element ID:"
                     << (*vIt).uniqueID << " from the TrackedVehicles\n";
          export_csv.close();
          #endif

          vIt = TrackedVehicles.erase(vIt);
        }
        else
        {
          ++vIt;
        }
      }
    }

    #ifdef ENABLE_DEBUG_MODE
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
    export_csv << "==========:\n";
    export_csv << "TrackedVehicles Final to Frame:" << frame_count << "\n";
    for(auto it = TrackedVehicles.begin(); it != TrackedVehicles.end(); ++it)
    {
      export_csv << "{ID:" << (*it).uniqueID << ", Class:" << (*it).vehicleClass
                 << ", Coordinates:[";
      for(auto coordIt = (*it).centerPoint.begin();
          coordIt != (*it).centerPoint.end(); ++coordIt)
          {
              export_csv << "(" << (*coordIt).x << ", " << (*coordIt).y << "),";
          }
      export_csv << "], Frames:[";
      for(auto frameIt = (*it).frameNo.begin(); frameIt != (*it).frameNo.end();
          ++frameIt)
          {
            export_csv << (*frameIt) << ", ";
          }
      export_csv << "]}\n";
    }
    export_csv.close();
    #endif
}

#ifdef ENABLE_DEBUG_MODE
void debugListFrame()
{
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
  export_csv << "==========\n";
  export_csv << "Frame ID:," << frame_count <<"\n";
  export_csv.close();
}

void debugListVehicle()
{
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
  export_csv << "Current vehicles final to the frame:" << frame_count << "\n";
  for(auto it = vehicles.begin(); it != vehicles.end(); ++it)
  {
    export_csv << "{UniqueID:" << (*it).detectionID
               << ", X-Center:" << (*it).x
               << ", Y-Center:" << (*it).y
               << ", X-Length:" << (*it).xDimension
               << ", Y-Length:" << (*it).yDimension
               << "}\n";
  }
  export_csv << "Previous vehicles to use in frame:" << frame_count + 1 << "\n";
  for(auto it = prev_vehicles.begin(); it!= prev_vehicles.end(); ++it)
  {
    export_csv << "{UniqueID:" << (*it).detectionID
               << ", X-Center:" << (*it).x
               << ", Y-Center:" << (*it).y
               << ", X-Length:" << (*it).xDimension
               << ", Y-Length:" << (*it).yDimension
               << "}\n";
  }
  export_csv.close();
}
#endif

void prepareNextFrame()
{
  if(vehicles.size()!=0)
  {
    prev_vehicles = vehicles;
    #ifdef ENABLE_DEBUG_MODE
    export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
    export_csv << "----------\n";
    export_csv << "Overwriting current frame vehicles to previous vehicles\n";
    export_csv.close();
    #endif
  }
  vehicles.clear();
  ppVehicleFrame.clear();

  #ifdef ENABLE_DEBUG_MODE
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/tracker_debugging.csv", std::ofstream::app);
  export_csv << "----------\n";
  export_csv << "Wiping current frame vehicles\n";
  export_csv.close();
  #endif

  EndTrackVehicles.clear();
  #ifdef ENABLE_DEBUG_MODE
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/ROI_debugging.csv", std::ofstream::app);
  export_csv << "----------\n";
  export_csv << "Wiping list of vehicles to track in current frame\n";
  export_csv.close();
  #endif

}

void generatePerspective()
{
  if(!runPerspective)
  {
    std::vector<cv::Point2f> roadPoints; //type must be Point2f
    std::vector<cv::Point2f> newImagePoints;
    roadPoints.push_back(cv::Point(716,270));
    roadPoints.push_back(cv::Point(1170,270));
    roadPoints.push_back(cv::Point(310,860));
    roadPoints.push_back(cv::Point(1530,860));
    newImagePoints.push_back(cv::Point(0,0));
    newImagePoints.push_back(cv::Point(690,0));
    newImagePoints.push_back(cv::Point(0,1220));
    newImagePoints.push_back(cv::Point(690,1220));
    ppMatrix = cv::getPerspectiveTransform(roadPoints, newImagePoints);
    runPerspective = true;
  }
  #ifdef ENABLE_PERSPECTIVE_FEED
  cv::warpPerspective(frame, ppImage, ppMatrix, ppImageSize);
  #endif
}
