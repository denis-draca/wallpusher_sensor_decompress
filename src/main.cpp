//#include "ros/ros.h"
#include "wallpusher_sensor_decompress/decompresser.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wallpusher_listener");
  ros::NodeHandle n;

  DECOMPRESSER decompress(n);


  ros::spin();
  ros::shutdown();

  return 0;
}
