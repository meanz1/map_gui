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
    cv::Mat gray;
    cv::Mat img_roi;
    int map_width, map_height;
    int map_x, map_y;

    std::vector<cv::Point> pointList; /// vector좌표 push해서 좌표가지고 사각형 그리고, +, - 구현하기

    float m2pixel;
    int a = 10;
    int b = 10;
    void initNode();

    int roi_x, roi_y, roi_height;
    float roi_width;
    void mapCallback(nav_msgs::OccupancyGridConstPtr map);
    void readMap();
    // void mapViewCallback()
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
    cv::Mat img = cv::imread("/home/minji/map_gui/src/Map2/Map1.pgm", 1);
}

void MODMAP::readMap()
{
    cv::Mat img = cv::imread("/home/minji/map_gui/src/Map2/Map1.pgm", 1);

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
    cv_ptr->encoding = "bgr8";
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
    cv_ptr->encoding = "bgr8";
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
    int width = root["width"].asInt();
    int height = root["height"].asInt();
    int x = root["x"].asInt();
    int y = root["y"].asInt();

    // std::vector<std::string> x_split = split(x_, '.');
    // std::vector<std::string> y_split = split(y_, '.');

    // map_x = std::stoi(x); //수정해야함
    // map_y = std::stoi(y);
    // map_width = std::stoi(width);
    // map_height = std::stoi(height);
    float big_size, small_size, resolution, w, h;
    cv::Mat img = cv::imread("/home/minji/map_gui/src/Map2/Map1.pgm", 1);
    cv::Mat img_origin = img.clone();

    // cv::Point left_top(x, y);
    // cv::Point left_bottom(x, y + 40);
    // cv::Point right_top(x + 40, y);
    // cv::Point right_bottom(x + 40, y + 40);
    if (img.cols >= img.rows)
    {
        big_size = img.cols;
        small_size = img.rows;
        resolution = width / big_size;
        w = width;
        h = small_size * resolution;
        std::cout << "first" << std::endl;
    }
    else
    {
        big_size = img.rows;
        small_size = img.cols;
        resolution = height / big_size;
        w = small_size * resolution;
        h = height;
        std::cout << "second" << std::endl;
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
        cv::resize(img_origin, img_origin, cv::Size(w, h));
        std::cout << resolution << std::endl;
        std::cout << w << std::endl;
        std::cout << h << std::endl;

        mapPublish(img_origin);
    }

    if (type == "map_draw")
    {
        int er = 0;
        a = 10;
        b = 10;

        try
        {
            er = 5;
            img_roi = img(cv::Rect(x, y, 40, 40));
            cv::resize(img_roi, img_roi, cv::Size(400, 400));
            std::cout << img_roi.channels() << std::endl;
            roi_x = x;
            roi_y = y;
            roi_height = 40;
            roi_width = 40;
            if (x + 40 > img.cols || y + 40 > img.rows)
            {
                throw er;
            }

            mapBigPublish(img_roi);
        }
        catch (int er)
        {
            std::cout << er << std::endl;
        }
    }

    if (type == "plus" || type == "minus")
    {

        int er = 0;
        if (type == "plus")
        {
            try
            {
                roi_x = roi_x + 10;
                roi_y = roi_y + 10;
                roi_height = roi_height - 2 * a;
                roi_width = roi_width - 2 * a;
                b = 10;
                er = 7;
                img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                cv::resize(img_roi, img_roi, cv::Size(400, 400));
                // cv::cvtColor(img_roi, gray, CV_GRAY2RGB);
                if (roi_x + 40 - a > img.cols || roi_y + 40 - a > img.rows)
                {
                    throw er;
                }

                mapBigPublish(img_roi);
                // a += 10;

                std::cout << roi_x << std::endl;
            }
            catch (int er)
            {
                std::cout << er << std::endl;
            }
        }

        if (type == "minus")
        {
            try
            {
                roi_x = roi_x - 10;
                roi_y = roi_y - 10;
                roi_height = roi_height + 2 * b;
                roi_width = roi_width + 2 * b;
                a = 10;
                er = 9;
                img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                cv::resize(img_roi, img_roi, cv::Size(400, 400));
                // cv::cvtColor(img_roi, gray, CV_GRAY2RGB);
                if (roi_x + 40 + b > img.cols || roi_y + 40 + b > img.rows)
                {
                    throw er;
                }
                std::cout << roi_x << std::endl;
                std::cout << roi_y << std::endl;
                std::cout << roi_width << std::endl;
                std::cout << 400 / roi_width << std::endl;
                mapBigPublish(img_roi);
                // b += 10;
            }
            catch (int er)
            {
                std::cout << er << std::endl;
            }
        }
    }

    if (type == "save")
    {
        for (int i = 0; i < img_roi.cols; i++)
        {
            for (int j = 0; j < img_roi.rows; j++)
            {
                std::cout << "c" << std::endl;
                //////////////////여기에 mat변환하는거야 !

                if (img_roi.at<cv::Vec3b>(i, j)[0] == 255)
                {
                    std::cout << "change pixel0" << std::endl;
                }
                if (img_roi.at<cv::Vec3b>(i, j)[1] == 255)
                {
                    std::cout << "change pixel1" << std::endl;
                }
                if (img_roi.at<cv::Vec3b>(i, j)[2] == 255)
                {
                    std::cout << "change pixel2" << std::endl;
                }
            }
        }
        std::cout << "hpp->save" << std::endl;
    }

    // if (type == "plus")
    // {
    //     int er = 0;

    //     try
    //     {
    //         er = 7;
    //         img_roi = img(cv::Rect(x, y, 40 - a, 40 - a));
    //         cv::resize(img_roi, img_roi, cv::Size(400, 400));
    //         if (x + 40 - a > img.cols || y + 40 - a > img.rows)
    //         {
    //             throw er;
    //         }

    //         mapBigPublish(img_roi);
    //         a = a - 20;
    //     }
    //     catch (int er)
    //     {
    //         std::cout << er << std::endl;
    //     }
    // }

    // if (type == "minus")
    // {
    //     int er = 0;

    //     try
    //     {
    //         er = 7;
    //         img_roi = img(cv::Rect(x, y, 40 + a, 40 + a));
    //         cv::resize(img_roi, img_roi, cv::Size(400, 400));
    //         if (x + 40 + a > img.cols || y + 40 + a > img.rows)
    //         {
    //             throw er;
    //         }

    //         mapBigPublish(img_roi);
    //         a = a + 20;
    //     }
    //     catch (int er)
    //     {
    //         std::cout << er << std::endl;
    //     }
    // }
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