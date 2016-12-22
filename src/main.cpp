//#include "ros/ros.h"
#include "wallpusher_sensor_decompress/decompresser.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  DECOMPRESSER decompress;


  ros::spin();

  return 0;
}
