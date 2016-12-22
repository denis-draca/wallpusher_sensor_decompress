#include "wallpusher_sensor_decompress/decompresser.h"


DECOMPRESSER::DECOMPRESSER(ros::NodeHandle &n):
    n_(n), it_(n)
{
    pub_left_ = it_.advertise("/wallpusher/decompressed/left",1);
    pub_right_= it_.advertise("/wallpushed/decompressed/right",1);

    sub_left_ = n_.subscribe("/wallpusher/raw_scan/left", 1, &DECOMPRESSER::left_scan, this);
    sub_right_= n_.subscribe("/wallpusher/raw_scan/right", 1, &DECOMPRESSER::right_scan, this);
}

void DECOMPRESSER::left_scan(const std_msgs::Int8MultiArrayConstPtr &input)
{

    if (input->layout.dim[0].label == "a" && !left_aDone)
    {
        for (int i = 0; i < input->layout.dim[0].size; i++)
        {
            input_scan_left_.push_back(input->data[i] + 128);
        }

        left_aDone = true;
        return;
    }

    if (left_aDone && input->layout.dim[0].label != "a")
    {
        for (int i = 0; i < input->data.size(); i++)
        {
           input_scan_left_.push_back(input->data.at(i) + 128);
        }

        left_aDone = false;
    }

    if(input_scan_left_.size() > 512)
    {
        ROS_INFO("TOO MANY DATA POINTS SENT -> %d", (int)input_scan_left_.size());
        return;
    }

//    cv::Mat image(32,16,CV_8U);
    cv::Mat image(32,16,CV_8U);


    std::cout << "------------------SCAN READING START------------------" <<std::endl;
    for (int i = 0; i <input_scan_left_.size(); i++)
    {
        std::cout << input_scan_left_.at(i) << " , ";
    }

    std::cout << std::endl << "------------------SCAN READING END------------------" <<std::endl;

    std::vector <uchar> v;
    bool next = false;

    for (int i = 0; i < input_scan_left_.size(); i++)
    {
        if(next)
        {
            next = false;
            continue;
        }

        if (input_scan_left_.at(i) == 0)
        {
            if (i == input_scan_left_.size() - 1)
            {
                v.push_back(0);
            }
            else
            {
                for(int y = 0 ; y < input_scan_left_.at(i + 1); y++)
                {
                    v.push_back(0);
                }
            }
            next = true;
        }
        else
        {
            v.push_back(input_scan_left_.at(i));
        }

    }

    input_scan_left_.clear();

    if(v.size() == 0)
    {
        ROS_INFO("DECOMPRESSED DATA is EMPTY -> %d", (int)v.size());
        return;
    }

    int max = find_max(v);

    double force = 0.0;

    int total_ADC = 0;

    for (int i = 0; i < v.size(); i++)
    {

        total_ADC += v.at(i);
    }

    force = exp((total_ADC + 558.73)/462.81);

    std::cout << "------------------TOTAL ADC START ------------------" <<std::endl<<std::endl;
    std::cout << "------------------ " << total_ADC << " ---------------------" << std::endl<<std::endl;
    std::cout << "------------------TOTAL ADC END------------------" <<std::endl<<std::endl;

    std::cout << "------------------TOTAL FORCE START ------------------" <<std::endl<<std::endl;
    std::cout << "------------------ " << force/9.81 << " ---------------------" << std::endl<<std::endl;
    std::cout << "------------------ TOTAT FORCE END------------------" <<std::endl<<std::endl;
    if (v.size() > 512)
    {
        ROS_INFO("DECOMPRESSED DATA HAS TOO MANY DATA POINTS -> %d", (int)v.size());
        return;
    }


    int pos = 0;
    for (int y = 0; y < image.rows ; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            if (pos >= v.size())
            {
                ROS_INFO("ACCESSING BEYOND THE DECOMPRESSED VECTOR pos -> %d  v.size() -> %d", (int)pos, (int)v.size());
                return;
            }
            if(v.at(pos) == max)
            {
                image.at<uchar>(y,x) = 255;
            }
            else
            {
                image.at<uchar>(y,x) = v.at(pos);
            }

            pos++;
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    pub_left_.publish(msg);
    cv::namedWindow("FOUND",cv::WINDOW_NORMAL);
    cv::imshow("FOUND",image);
    cv::waitKey(3);
}

void DECOMPRESSER::right_scan(const std_msgs::Int16MultiArrayConstPtr &input)
{

    if (input->layout.dim[0].label == "a" && !right_aDone)
    {
        for (int i = 0; i < input->layout.dim[0].size; i++)
        {
            input_scan_right_.push_back(input->data[i] + 128);
        }

        right_aDone = true;
        return;
    }

    if (right_aDone && input->layout.dim[0].label != "a")
    {
        for (int i = 0; i < input->data.size(); i++)
        {
           input_scan_right_.push_back(input->data.at(i) + 128);
        }

        right_aDone = false;
    }

    if(input_scan_right_.size() > 512)
    {
        ROS_INFO("TOO MANY DATA POINTS SENT -> %d", (int)input_scan_right_.size());
        return;
    }

//    cv::Mat image(32,16,CV_8U);
    cv::Mat image(32,16,CV_8U);


    std::cout << "------------------SCAN READING START------------------" <<std::endl;
    for (int i = 0; i <input_scan_right_.size(); i++)
    {
        std::cout << input_scan_right_.at(i) << " , ";
    }

    std::cout << std::endl << "------------------SCAN READING END------------------" <<std::endl;

    std::vector <uchar> v;
    bool next = false;

    for (int i = 0; i < input_scan_right_.size(); i++)
    {
        if(next)
        {
            next = false;
            continue;
        }

        if (input_scan_right_.at(i) == 0)
        {
            if (i == input_scan_right_.size() - 1)
            {
                v.push_back(0);
            }
            else
            {
                for(int y = 0 ; y < input_scan_right_.at(i + 1); y++)
                {
                    v.push_back(0);
                }
            }
            next = true;
        }
        else
        {
            v.push_back(input_scan_right_.at(i));
        }

    }

    input_scan_right_.clear();

    if(v.size() == 0)
    {
        ROS_INFO("DECOMPRESSED DATA is EMPTY -> %d", (int)v.size());
        return;
    }

    int max = find_max(v);

    double force = 0.0;

    int total_ADC = 0;

    for (int i = 0; i < v.size(); i++)
    {

        total_ADC += v.at(i);
    }

    force = exp((total_ADC + 558.73)/462.81);

    std::cout << "------------------TOTAL ADC START ------------------" <<std::endl<<std::endl;
    std::cout << "------------------ " << total_ADC << " ---------------------" << std::endl<<std::endl;
    std::cout << "------------------TOTAL ADC END------------------" <<std::endl<<std::endl;

    std::cout << "------------------TOTAL FORCE START ------------------" <<std::endl<<std::endl;
    std::cout << "------------------ " << force/9.81 << " ---------------------" << std::endl<<std::endl;
    std::cout << "------------------ TOTAT FORCE END------------------" <<std::endl<<std::endl;
    if (v.size() > 512)
    {
        ROS_INFO("DECOMPRESSED DATA HAS TOO MANY DATA POINTS -> %d", (int)v.size());
        return;
    }


    int pos = 0;
    for (int y = 0; y < image.rows ; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            if (pos >= v.size())
            {
                ROS_INFO("ACCESSING BEYOND THE DECOMPRESSED VECTOR pos -> %d  v.size() -> %d", (int)pos, (int)v.size());
                return;
            }
            if(v.at(pos) == max)
            {
                image.at<uchar>(y,x) = 255;
            }
            else
            {
                image.at<uchar>(y,x) = v.at(pos);
            }


            pos++;
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    pub_right_.publish(msg);

    cv::namedWindow("FOUND_RIGHT",cv::WINDOW_NORMAL);
    cv::imshow("FOUND_RIGHT",image);
    cv::waitKey(3);
}


int DECOMPRESSER::find_max(std::vector<uchar> &v)
{

    int max_value = v.front();

    for(int i = 0; i < v.size(); i++)
    {

        if (v.at(i) > max_value)
        {
            max_value = v.at(i);
        }
    }

    return max_value;
}

