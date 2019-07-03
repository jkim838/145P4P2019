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
#include "boost/date_time/posix_time/posix_time.hpp"

std_msgs::Int8 vehicle_count;
int vehicle_count_integer;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  printf("keystroke: %d", c);
  return c;
}

void count_object_no(const std_msgs::Int8::ConstPtr& count_value){
  vehicle_count.data = count_value->data;
  vehicle_count_integer = vehicle_count.data;
  //ROS_INFO("RECEIVED: : %d\n", count_value->data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "traffic_analyzer");

  auto start_time = std::chrono::system_clock::now();
  std::time_t start_time_formatted = std::chrono::system_clock::to_time_t(start_time);

  ros::NodeHandle ta_nh;

  // Subscriber Declaration
  ros::Subscriber ta_coordinate_sub = ta_nh.subscribe("/darknet_ros/found_object", 10, count_object_no);
  // ros::Subscriber ta_array_size_sub = ta_nh.subscribe("/TopicName2", 10, function_name2);

  // Publisher Declaration
  // ros::Publisher ta_pub = ta_nh.advertise<std_msgs::Uint64>("/TopicName", 10)
  ros::Rate loop_rate(10);

  while(ros::ok()){
    ros::spinOnce();
    int keystroke = getch();

    if(keystroke == 's'){
     printf("FOUND A KEYSTROKE: %d\n", keystroke);
     auto record_time = std::chrono::system_clock::now();
     std::time_t record_time_formatted = std::chrono::system_clock::to_time_t(record_time);
     std::string start_time_string = std::ctime(&start_time_formatted);
     std::string record_time_string = std::ctime(&record_time_formatted);
     std::ofstream export_csv("/home/master/catkin_ws/src/145P4P2019/csv/"+start_time_string+".csv");
     printf("FILE IS OPEN: %d \n", export_csv.is_open());
     export_csv << "Start:," << start_time_string;
     export_csv << "End:," << record_time_string;
     export_csv << "Count:," << vehicle_count_integer;
     export_csv.close();
     printf("FILE IS CLOSED: %d \n", export_csv.is_open());

     printf("END OF FILE\n");
    }

    // Publish info
    // ta_pub.publish(value_to_publish);

    loop_rate.sleep();
  }

  return 0;

}
