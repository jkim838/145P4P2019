#include <ros/ros.h>
#include <stdio.h>
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

/*** Function Prototypes ***/
char getch();
void count_object_no(const std_msgs::Int8::ConstPtr& count_value);
void extract_bounding_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "traffic_analyzer");

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

  while(ros::ok()){
    ros::spinOnce();
    int keystroke = getch();

    if(keystroke == 's'){

     printf("FOUND A KEYSTROKE: %d\n", keystroke);

     // Get system time at the moment of save...
     auto record_time = std::chrono::system_clock::now();
     std::time_t record_time_formatted = std::chrono::system_clock::to_time_t(record_time);
     std::string record_time_string = std::ctime(&record_time_formatted);

     //std::string start_time_string = std::ctime(&start_time_formatted);

     //std::ofstream export_csv("/home/master/catkin_ws/src/145P4P2019/csv/"+start_time_string+".csv");
     //export_csv << "Start:," << start_time_string;
     export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv", std::ofstream::app);
     export_csv << "Start:," << start_time_string; //Record begin time...
     export_csv << "End:," << record_time_string;
     export_csv.close();
     printf("SAVING DATA: %d \n", export_csv.is_open());
     std::string rename_file_to = "/home/master/catkin_ws/src/145P4P2019/csv/"+record_time_string+".csv";
     std::rename("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv",rename_file_to.c_str());
     std::ofstream generate_new_csv("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv");
     generate_new_csv.close();

    }

    // Publish info
    // ta_pub.publish(value_to_publish);

    loop_rate.sleep();
  }

  return 0;

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
  std::ofstream export_csv;
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv", std::ofstream::app);
  export_csv << "Count:," << vehicle_count_integer << "\n";
  export_csv.close();

}

void extract_bounding_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox){

  // Builds well, but needs fixing in terms of logic.
  int min_x = bbox->bounding_boxes[2].xmin;
  int min_y = bbox->bounding_boxes[3].ymin;
  int max_x = bbox->bounding_boxes[4].xmax;
  int max_y = bbox->bounding_boxes[5].ymax;
  std::string vehicle_class = bbox->bounding_boxes[0].Class;
  int x_dimension_bbox = (max_x - min_x);
  int y_dimension_bbox = (max_y - min_y);
  int x_center = min_x + x_dimension_bbox/2;
  int y_center = min_y + y_dimension_bbox/2;
  std::ofstream export_csv;
  export_csv.open("/home/master/catkin_ws/src/145P4P2019/csv/active_record.csv", std::ofstream::app);
  export_csv << "Detection ID:," << detection_id <<"\n";
  export_csv << "Vehicle Class:," << vehicle_class << "\n";
  export_csv << "X-Length:," << x_dimension_bbox << "\n";
  export_csv << "Y-Length:," << y_dimension_bbox << "\n";
  export_csv << "X-Center:," << x_center << "\n";
  export_csv << "Y-Center:," << y_center << "\n";
  export_csv.close();
  detection_id++;

}
