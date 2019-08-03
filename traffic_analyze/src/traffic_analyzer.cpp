#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <stdio.h>
#include <signal.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO
#include <chrono>
#include <ctime>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <traffic_tracker/trackerOutput.h>
#include <traffic_tracker/point2f.h>
#include <traffic_tracker/perspectiveVehicle.h>

std::ofstream export_csv;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

/*** Function Prototypes ***/
char getch();
void analyze_trackerOutput(const traffic_tracker::trackerOutput::ConstPtr& trackerOutput);
void SIGNALHandler(int sig);
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traffic_analyzer");
  signal((SIGINT|SIGTERM), SIGNALHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // Get system time at the moment the node was launched...
  auto start_time = std::chrono::system_clock::now();
  std::time_t start_time_formatted = std::chrono::system_clock::to_time_t(start_time);
  std::string start_time_string = std::ctime(&start_time_formatted);
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv",
                  std::ofstream::app);
  export_csv.close();
  ros::NodeHandle ta_nh;

  // Subscriber Declaration
  ros::Subscriber ta_trackerOutput_sub =
  ta_nh.subscribe("/traffic_tracker/trackerOutput", 1000, analyze_trackerOutput);

  // Publisher Declaration
  // ros::Publisher ta_pub = ta_nh.advertise<std_msgs::Uint64>("/TopicName", 10)
  ros::Rate loop_rate(1000);

  while(!g_request_shutdown)
  {
    ros::spinOnce();
    int keystroke = getch();

    if(keystroke == 's')
    {
     ROS_INFO("SAVING DATA...");
     // Get system time at the moment of save...
     auto record_time = std::chrono::system_clock::now();
     std::time_t record_time_formatted = std::chrono::system_clock::to_time_t(record_time);
     std::string record_time_string = std::ctime(&record_time_formatted);
     export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv", std::ofstream::app);
     export_csv << "Start:," << start_time_string; //Record begin time...
     export_csv << "End:," << record_time_string;
     export_csv.close();
     printf("SAVING DATA: %d \n", export_csv.is_open());
     std::string rename_file_to = "/home/master/catkin_ws/src/145P4P2019/csv/"+record_time_string+".csv";
     std::rename("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv",
                 rename_file_to.c_str());
     export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv");
     export_csv.close();
    }
    loop_rate.sleep();
  }

  std::remove("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv");
  ros::shutdown();

}

// Replacement SIGINT handler
void SIGNALHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }
  result = ros::xmlrpc::responseInt(1, "", 0);
}

char getch()
{
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0)
      ROS_ERROR("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0)
      ROS_ERROR("tcsetattr ICANON");

  if(rv == -1)
      ROS_ERROR("select");
  else if(rv == 0)
  {
      //do nothing;
  }
  else
      read(filedesc, &buff, len );

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
      ROS_ERROR ("tcsetattr ~ICANON");
  return (buff);
}

void analyze_trackerOutput(const traffic_tracker::trackerOutput::ConstPtr& trackerOutput)
{
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv",
                  std::ofstream::app);
  export_csv << "FRAME:" << trackerOutput->frameCount << "\n"
             << "Vehicles in Frame:" << trackerOutput->trackerOutput.size() <<"\n"
             << "Total Vehicle Count:" << trackerOutput->vehicleCount<< "\n";
  for(auto toIt = trackerOutput->trackerOutput.begin();
      toIt != trackerOutput->trackerOutput.end(); ++toIt)
  {
    float yFront = (float)((*toIt).centerPoint.front().y);
    float yBack = (float)((*toIt).centerPoint.back().y);
    float yPxDiff = yFront-yBack;
    float yMeterPerPixel = 40.0/825.0;
    float frameBack = (float)((*toIt).frameNo.back());
    float frameFront = (float)((*toIt).frameNo.front());
    float frameDiff = frameBack - frameFront;
    float frameTime = frameDiff/30.0;
    float yVelocity = (yPxDiff * yMeterPerPixel)/frameTime * 3.6;
    float xFront = (float)((*toIt).centerPoint.front().x);
    float xBack = (float)((*toIt).centerPoint.back().x);
    float xPxDiff = xFront-xBack;
    float xMeterPerPixel = 10.0/690.0;
    float xVelocity = (xPxDiff * xMeterPerPixel)/frameTime * 3.6;
    export_csv << "ID:" << (*toIt).uniqueID
               << ",Class:" << (*toIt).vehicleClass
               << ", Y-Velocity:" << yVelocity
               << ", X-Velocity:" << xVelocity;
    if(yVelocity >= 120)
    {
      export_csv << ", OVERSPEED";
    }
    if(abs(xVelocity) >= 2){
      export_csv << ",";
      if(abs(xVelocity >= 3)){
        export_csv << " AGGRESSIVE";
      }
      export_csv << " LANE CHANGE";
    }
    export_csv <<"\n";
  }
  export_csv.close();
}
