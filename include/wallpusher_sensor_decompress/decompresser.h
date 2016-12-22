#ifndef DECOMPRESSER_H
#define DECOMPRESSER_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8MultiArray.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <vector>

class DECOMPRESSER
{
private:
    //Node Handle
    ros::NodeHandle n_;

    //Publishers
    image_transport::Publisher pub_left_;
    image_transport::Publisher pub_right_;

    image_transport::ImageTransport it_;

    //Subscribers
    ros::Subscriber sub_left_;
    ros::Subscriber sub_right_;

    //Vectors
    std::vector<int16_t> input_scan_left_;
    std::vector<int16_t> input_scan_right_;
    //Booleans
    bool left_aDone = false;
    bool right_aDone = false;

private:
    void left_scan(const std_msgs::Int8MultiArrayConstPtr &input);
    void right_scan(const std_msgs::Int16MultiArrayConstPtr &input);

    int find_max(std::vector<uchar> &v);

private:
    DECOMPRESSER();

public:
    DECOMPRESSER(ros::NodeHandle &n);
};

#endif // DECOMPRESSER_H
