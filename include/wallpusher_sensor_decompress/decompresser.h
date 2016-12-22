#ifndef DECOMPRESSER_H
#define DECOMPRESSER_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int16MultiArray.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>

class DECOMPRESSER
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub2_;
    ros::Subscriber sub_;
    ros::Subscriber sub2_;

    std::vector<int16_t> input_scan;

    bool aDone = false;

private:
    void left_scan(const std_msgs::Int16MultiArrayConstPtr &input);
    int find_max(std::vector<uchar> &v);

public:
    DECOMPRESSER();
};

#endif // DECOMPRESSER_H
