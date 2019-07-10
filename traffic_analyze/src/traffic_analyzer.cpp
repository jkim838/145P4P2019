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
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO
#include <chrono>
#include <ctime>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

int vehicle_count_integer;
int detection_id = 0;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

/*** Function Prototypes ***/
char getch();
void count_object_no(const std_msgs::Int8::ConstPtr& count_value);
void extract_bounding_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
void SIGNALHandler(int sig);
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

int main(int argc, char **argv){

  ros::init(argc, argv, "traffic_analyzer");
  signal((SIGINT|SIGTERM), SIGNALHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // Get system time at the moment the node was launched...
  auto start_time = std::chrono::system_clock::now();
  std::time_t start_time_formatted = std::chrono::system_clock::to_time_t(start_time);
  std::string start_time_string = std::ctime(&start_time_formatted);
  std::ofstream export_csv("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv");
  export_csv.close();
  ros::NodeHandle ta_nh;

  // Subscriber Declaration
  ros::Subscriber ta_frame_vehicle_count_sub = ta_nh.subscribe("/darknet_ros/found_object", 1000, count_object_no);
  ros::Subscriber ta_bounding_box_sub = ta_nh.subscribe("/darknet_ros/bounding_boxes", 1000, extract_bounding_box);

  // Publisher Declaration
  // ros::Publisher ta_pub = ta_nh.advertise<std_msgs::Uint64>("/TopicName", 10)
  ros::Rate loop_rate(10);

  while(!g_request_shutdown){

    ros::spinOnce();
    int keystroke = getch();

    if(keystroke == 's'){

     printf("FOUND A KEYSTROKE: %d\n", keystroke);

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
     std::rename("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv",rename_file_to.c_str());
     std::ofstream generate_new_csv("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv");
     generate_new_csv.close();
     detection_id = 0;

    }

    // Publish info
    // ta_pub.publish(value_to_publish);

    loop_rate.sleep();

  }

  std::remove("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv");
  ros::shutdown();

}

// Replacement SIGINT handler
void SIGNALHandler(int sig){

  g_request_shutdown = 1;

}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result){

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

char getch(){

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
  else if(rv == 0){
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

void count_object_no(const std_msgs::Int8::ConstPtr& count_value){

  vehicle_count_integer = count_value->data;
  // std::ofstream export_csv;
  // export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv", std::ofstream::app);
  // export_csv << "Count:," << vehicle_count_integer << "\n";
  // export_csv.close();

}

void extract_bounding_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){

  // Negative coordinate error still persists....
  const long int min_x = bbox->bounding_boxes[0].xmin;
  const long int min_y = bbox->bounding_boxes[0].ymin;
  const long int max_x = bbox->bounding_boxes[0].xmax;
  const long int max_y = bbox->bounding_boxes[0].ymax;
  std::string vehicle_class = bbox->bounding_boxes[0].Class;
  const long int x_dimension_bbox = (max_x - min_x);
  const long int y_dimension_bbox = (max_y - min_y);
  const long int x_center = min_x + x_dimension_bbox/2;
  const long int y_center = min_y + y_dimension_bbox/2;
  std::ofstream export_csv;
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv", std::ofstream::app);
  export_csv << "Detection ID:," << detection_id <<"\n";
  export_csv << "Count:," << vehicle_count_integer << "\n";
  export_csv << "Vehicle Class:," << vehicle_class << "\n";
  export_csv << "X-min:," << min_x << "\n";
  export_csv << "Y-min:," << min_y << "\n";
  export_csv << "X-max:," << max_x << "\n";
  export_csv << "Y-max:," << max_y << "\n";
  export_csv << "X-Length:," << x_dimension_bbox << "\n";
  export_csv << "Y-Length:," << y_dimension_bbox << "\n";
  export_csv << "X-Center:," << x_center << "\n";
  export_csv << "Y-Center:," << y_center << "\n";
  export_csv.close();
  detection_id++;

}
