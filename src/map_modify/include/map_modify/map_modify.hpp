#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
#include <cstdio>
#include <iostream>
#include <string>
#include <cstring>
#include <fstream>
#include <vector>
#include <thread>
#include <time.h>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>

#include <jsoncpp/json/json.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#define TRUE 1
#define FALSE 0

class MODMAP
{
public:
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    ros::Subscriber sub, sub2, sub3, sub4;
    ros::Publisher pub_pgmsize;
    image_transport::Publisher map_pub, map_pub_big;
    cv::Mat Mapimage, globalMap, EditedMap, MergedMap, FilteredMap, Map4path;
    int map_width, map_height;
    float map_x, map_y;

    float m2pixel;

    void initNode();

    void mapCallback(nav_msgs::OccupancyGridConstPtr map);
    void readMap();
    void btnCallback(const std_msgs::String::ConstPtr &msg);
    void jsonCallback(const std_msgs::String::ConstPtr &msg);
    void mapPublish(cv::Mat image);
    void mapBigPublish(cv::Mat image);

    MODMAP()
        : it(n)
    {
        initNode();

        // readMap();
        //  readPGM(&a);
    }
};

void MODMAP::initNode()
{
    sub = n.subscribe("/map", 1, &MODMAP::mapCallback, this);
    sub2 = n.subscribe("/btnInput", 1, &MODMAP::btnCallback, this);
    sub3 = n.subscribe("/btnInput", 1, &MODMAP::jsonCallback, this);
    sub4 = n.subscribe("/mappos", 1, &MODMAP::jsonCallback, this);
    map_pub = it.advertise("map_pgm", 1);
    map_pub_big = it.advertise("map_big", 1);

    ros::Rate loop_rate(10);
    cv::Mat img = cv::imread("/home/minji/map_gui/src/map/Map1.pgm", cv::IMREAD_UNCHANGED);
}

void MODMAP::readMap()
{
    cv::Mat img = cv::imread("/home/minji/map_gui/src/map/Map1.pgm", cv::IMREAD_UNCHANGED);

    if (!img.data)
    {
        std::cout << "no" << std::endl;
    }
    else
    {
        cv::namedWindow("image_window");
        cv::imshow("image_window", img);
        cv::waitKey(0);
    }
}

void MODMAP::btnCallback(const std_msgs::String::ConstPtr &msg) // json
{
    std::string button = msg->data;
    std::cout << button << std::endl;
    if (button == "get_map")
    {
        std::cout << "oh yeah" << std::endl;
        // mapPublishOnce(Mapimage);
    }
}

void MODMAP::mapPublish(cv::Mat image)
{
    ros::Time time = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->header.frame_id = "";
    cv_ptr->header.seq = 1;
    cv_ptr->header.stamp = time;
    cv_ptr->encoding = "mono8";
    cv_ptr->image = image;

    map_pub.publish(cv_ptr->toImageMsg());
}

void MODMAP::mapBigPublish(cv::Mat image)
{
    ros::Time time = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->header.frame_id = "";
    cv_ptr->header.seq = 1;
    cv_ptr->header.stamp = time;
    cv_ptr->encoding = "mono8";
    cv_ptr->image = image;

    map_pub_big.publish(cv_ptr->toImageMsg());
}

void MODMAP::jsonCallback(const std_msgs::String::ConstPtr &msg)
{
    Json::Value root;
    Json::Value Position;
    Json::Reader reader;
    reader.parse(msg->data, root);
    std::string type = root["type"].asString(); // get_map
    std::string width = root["width"].asString();
    std::string height = root["height"].asString();
    std::string x_ = root["x"].asString();
    std::string y_ = root["y"].asString();

    map_x = std::stof(x_); //수정해야함
    map_y = std::stof(y_);
    map_width = std::stoi(width);
    map_height = std::stoi(height);
    float big_size, small_size, resolution, w, h;
    cv::Mat img = cv::imread("/home/minji/map_gui/src/map/Map1.pgm", cv::IMREAD_UNCHANGED);
    cv::Mat img_roi;
    img_roi = img(cv::Rect(map_x - 5, map_y - 5, 10, 10));

    if (img.cols >= img.rows)
    {
        big_size = img.cols;
        small_size = img.rows;
        resolution = map_width / big_size;
        w = map_width;
        h = small_size * resolution;
    }
    else
    {
        big_size = img.rows;
        small_size = img.cols;
        resolution = map_height / big_size;
        w = big_size * resolution;
        h = map_height;
    }
    pub_pgmsize = n.advertise<std_msgs::Float32MultiArray>("mapsize", 100);
    std_msgs::Float32MultiArray mapsize;

    if (type == "get_map")
    {
        mapsize.data.push_back(w);
        mapsize.data.push_back(h);
        mapsize.data.push_back(resolution);
        std::cout << mapsize << std::endl;
        pub_pgmsize.publish(mapsize);
        cv::resize(img, img, cv::Size(w, h));
        std::cout << resolution << std::endl;
        std::cout << w << std::endl;
        std::cout << h << std::endl;

        mapPublish(img);
    }

    if (type == "map_draw")
    {

        std::cout << "big pub" << std::endl;
        mapBigPublish(img_roi);
        std::cout << "big pub" << std::endl;
    }
}

void MODMAP::mapCallback(nav_msgs::OccupancyGridConstPtr map)
{
    m2pixel = 1.0 / map->info.resolution;
    cv::Mat init_image((int)map->info.height, (int)map->info.width, CV_8UC3);
    if (Mapimage.empty())
        Mapimage = cv::Mat((int)map->info.height, (int)map->info.width, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i < map->info.width; i++)
    {
        for (int j = 0; j < map->info.height; j++)
        {
            if (map->data[j * (int)map->info.width + i] == 0)
            {
                std::cout << "000" << std::endl;
                init_image.at<cv::Vec3b>(j, i)[0] = 255;
                init_image.at<cv::Vec3b>(j, i)[1] = 255;
                init_image.at<cv::Vec3b>(j, i)[2] = 255;
            }
            else if (map->data[j * (int)map->info.width + i] == 100)
            {
                init_image.at<cv::Vec3b>(j, i)[0] = 0; // 0
                init_image.at<cv::Vec3b>(j, i)[1] = 0;
                init_image.at<cv::Vec3b>(j, i)[2] = 0;
            }
            else
            {
                init_image.at<cv::Vec3b>(j, i)[0] = 127; // 127s
                init_image.at<cv::Vec3b>(j, i)[1] = 127;
                init_image.at<cv::Vec3b>(j, i)[2] = 127;
            }
        }
    }
}