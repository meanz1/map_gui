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
    ros::Subscriber sub, sub2, sub3;
    ros::Publisher pub_pgmsize;
    image_transport::Publisher map_pub;
    cv::Mat Mapimage, globalMap, EditedMap, MergedMap, FilteredMap, Map4path;
    int map_width, map_height;

    float m2pixel;

    void initNode();

    void mapCallback(nav_msgs::OccupancyGridConstPtr map);
    void readMap();
    void btnCallback(const std_msgs::String::ConstPtr &msg);
    void jsonCallback(const std_msgs::String::ConstPtr &msg);
    void mapPublish(cv::Mat image);

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
    map_pub = it.advertise("map_pgm", 1);

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

void MODMAP::jsonCallback(const std_msgs::String::ConstPtr &msg)
{
    Json::Value root;
    Json::Value Position;
    Json::Reader reader;
    reader.parse(msg->data, root);
    std::string type = root["type"].asString(); // get_map
    std::string width = root["width"].asString();
    std::string height = root["height"].asString();
    map_width = std::stoi(width);
    map_height = std::stoi(height);
    float big_size, small_size, resolution, w, h;
    cv::Mat img = cv::imread("/home/minji/map_gui/src/map/Map1.pgm", cv::IMREAD_UNCHANGED);

    if (img.cols >= img.rows)
    {
        big_size = img.cols;
        small_size = img.rows;
        resolution = map_width / big_size;
        w = map_width;
        h = map_height * resolution;
    }
    else
    {
        big_size = img.rows;
        small_size = img.cols;
        resolution = map_height / big_size;
        w = map_width * resolution;
        h = map_height;
    }
    pub_pgmsize = n.advertise<std_msgs::Float32MultiArray>("mapsize", 100);
    std_msgs::Float32MultiArray mapsize;
    mapsize.data.clear();

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

        // cv::imshow("a", img);
        // cv::waitKey(0);

        mapPublish(img);
    }
}

// int MODMAP::readPGM(PGMImage *img){
//     ros::Rate loop_rate(10);
//     FILE* fp = fopen("/home/cona/map_gui/src/map/Map1.pgm", "r");
//     if(fp == NULL){
// 		fprintf(stderr, "파일을 열 수 없습니다 : %s\n", "Map1.pgm");
// 		return FALSE;
// 	}
//     fscanf(fp, "%c %c", &img->M    , &img->N     );   // 매직넘버 읽기
//     if(img->M != 'P' || img->N != '5'){
// 		fprintf(stderr, "PGM 이미지 포멧이 아닙니다 : %c%c\n", img->M, img->N);
// 		return FALSE;
// 	}

//     fscanf(fp, "%d %d", &img->width, &img->height);   // 가로, 세로 읽기
//     fscanf(fp, "%d"   , &img->max                );	// 최대명암도 값

// 	if(img->max != 255){
// 		fprintf(stderr, "올바른 이미지 포멧이 아닙니다.\n");
//         return FALSE;
//     }

//     // <-- 메모리 할당
//     img->pixels = (unsigned char**)calloc(img->height, sizeof(unsigned char*));

//     for(int i=0; i<img->height; i++){
//         img->pixels[i] = (unsigned char*)calloc(img->width, sizeof(unsigned char));
//     }
//     // -->

//     // <-- pbm 파일로부터 픽셀값을 읽어서 할당한 메모리에 load
//     int tmp;
//     for(int i=0; i<img->height; i++){
//         for(int j=0; j<img->width; j++){
//             fscanf(fp, "%d", &tmp);
//             if (tmp != 0){
//                 printf((const char*)tmp);
//             }
//             std::cout << tmp << std::endl;
//             img->pixels[i][j] = (unsigned char)tmp;
//         }
//     }

//     fclose(fp); // 더 이상 사용하지 않는 파일을 닫아 줌

//     return TRUE;
// }

// void MODMAP::closePGM(PGMImage* img)
// {
// 	for(int i=0; i<img->height; i++){
// 		free(img->pixels[i]);
// 	}

// 	free(img->pixels);
// }

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