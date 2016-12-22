#include "wallpusher_sensor_decompress/decompresser.h"


DECOMPRESSER::DECOMPRESSER()
{
    pub_ = n_.advertise<sensor_msgs::Image>("/wallpusher/decompressed/left",1);
    pub2_= n_.advertise<sensor_msgs::Image>("/wallpushed/decompressed/right",1);

    sub_ = n_.subscribe("/wallpusher/raw_scan/left", 1, &DECOMPRESSER::left_scan, this);
}

void DECOMPRESSER::left_scan(const std_msgs::Int16MultiArrayConstPtr &input)
{

    if (input->layout.dim[0].label == "a" && !aDone)
    {
        for (int i = 0; i < input->layout.dim[0].size; i++)
        {
            input_scan.push_back(input->data[i]);
        }

        aDone = true;
        return;
    }

    if (aDone && input->layout.dim[0].label != "a")
    {
        for (int i = 0; i < input->data.size(); i++)
        {
            input_scan.push_back(input->data.at(i));
        }

        aDone = false;
    }

    if(input_scan.size() > 512)
    {
        ROS_INFO("TOO MANY DATA POINTS SENT -> %d", input_scan.size());
        return;
    }

//    cv::Mat image(32,16,CV_8U);
    cv::Mat image(32,16,CV_8U);


    std::cout << "------------------SCAN READING START------------------" <<std::endl;
    for (int i = 0; i < input_scan.size(); i++)
    {
        std::cout << input_scan.at(i) << " , ";
    }

    std::cout << std::endl << "------------------SCAN READING END------------------" <<std::endl;

    std::vector <uchar> v;
    bool next = false;

    for (int i = 0; i < input_scan.size(); i++)
    {
        if(next)
        {
            next = false;
            continue;
        }

        if (input_scan.at(i) == 0)
        {
            if (i == input_scan.size() - 1)
            {
                v.push_back(0);
            }
            else
            {
                for(int y = 0 ; y < input_scan.at(i + 1); y++)
                {
                    v.push_back(0);
                }
            }
            next = true;
        }
        else
        {
            v.push_back(input_scan.at(i));
        }

    }

    input_scan.clear();

    if(v.size() == 0)
    {
        ROS_INFO("DECOMPRESSED DATA is EMPTY -> %d", v.size());
        return;
    }

    int max = find_max(v);

    double force = 0.0;

    int total_ADC = 0;

    for (int i = 0; i < v.size(); i++)
    {
//        if (v.at(i) != 0)
//        {
//            force += exp((((double)v.at(i))+ 63.622)/47.722);
//        }

        total_ADC += v.at(i);
    }

    force = exp((total_ADC + 558.73)/462.81);

//    force = exp((max +63.622)/47.722);

    std::cout << "------------------TOTAL ADC START ------------------" <<std::endl<<std::endl;
    std::cout << "------------------ " << total_ADC << " ---------------------" << std::endl<<std::endl;
    std::cout << "------------------TOTAL ADC END------------------" <<std::endl<<std::endl;

    std::cout << "------------------TOTAL FORCE START ------------------" <<std::endl<<std::endl;
    std::cout << "------------------ " << force/9.81 << " ---------------------" << std::endl<<std::endl;
    std::cout << "------------------ TOTAT FORCE END------------------" <<std::endl<<std::endl;
    if (v.size() > 512)
    {
        ROS_INFO("DECOMPRESSED DATA HAS TOO MANY DATA POINTS -> %d", v.size());
        return;
    }


    int pos = 0;
    for (int y = 0; y < image.rows ; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            if (pos >= v.size())
            {
                ROS_INFO("ACCESSING BEYOND THE DECOMPRESSED VECTOR pos -> %d  v.size() -> %d", pos, v.size());
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
    input_scan.clear();
    cv::namedWindow("FOUND",cv::WINDOW_NORMAL);
    cv::imshow("FOUND",image);
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

