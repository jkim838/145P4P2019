#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "traffic_analyzer");
  ros::NodeHandle ta_nh;

  // Subscriber Declaration
  // ros::Susbscriber ta_coordinate_sub = ta_nh.subscribe("/TopicName1", 10, function_name1);
  // ros::Subscriber ta_array_size_sub = ta_nh.subscribe("/TopicName2", 10, function_name2);

  // Publisher Declaration
  // ros::Publisher ta_pub = ta_nh.advertise<std_msgs::Uint64>("/TopicName", 10)
  ros::Rate loop_rate(10);

  while(ros::ok()){
    ros::spinOnce();
    int keystroke = getch();

    if(keystroke == 's'){
     printf("FOUND A KEYSTROKE: %d", keystroke);
     std::ofstream myfile("example.csv");

     printf("FILE IS OPEN: %d \n", myfile.is_open());
     myfile << "This is the first cell in the first column.\n";
     myfile << "a,b,c,\n";
     myfile << "c,s,v,\n";
     myfile << "1,2,3.456\n";
     myfile << "semi;colon";
     myfile.close();
     printf("FILE IS CLOSED: %d \n", myfile.is_open());

     printf("END OF FILE");
    }

    // Publish info
    // ta_pub.publish(value_to_publish);

    loop_rate.sleep();
  }

  return 0;

}
