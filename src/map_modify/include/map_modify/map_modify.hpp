#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float32MultiArray.h"
#include <boost/algorithm/string.hpp>

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
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#define TRUE 1
#define FALSE 0

class MODMAP
{
public:
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    ros::Subscriber sub, sub2, sub3, sub4;
    ros::Publisher pub_pgmsize, occ_pub;
    image_transport::Publisher map_pub, map_pub_big;
    cv::Mat Mapimage, globalMap, EditedMap, MergedMap, FilteredMap, Map4path;
    cv::Mat img;
    cv::Mat img_roi, img_origin, img_reset;
    std::string file_path = "/home/minji/map_gui/src/data/CoNA/";
    // cv::Mat color_img = cv::imread("/home/minji/map_gui/src/Map2/stMap.pgm", 1);
    // cv::Mat color_img = cv::imread(file_path, 1);
    cv::Mat color_img;
    nav_msgs::OccupancyGrid pgm_occ;
    int map_width, map_height;
    int map_x, map_y;
    int square_j, erase_j;
    float roi_res;
    int threshold_occupied = 65;
    int threshold_free = 25;

    int plus_cnt = 0;
    int minus_cnt = 0;

    std::mutex mtx;
    // std::string directory_path = "/home/minji/map_gui/src/data/CoNA/";
    std::string directory_path;
    std::string filename = "Map1";
    std::vector<cv::Point> pointList_line; /// vector좌표 push해서 좌표가지고 사각형 그리고, +, - 구현하기
    std::vector<cv::Point> pointList_square;
    std::vector<cv::Point> pointList_erase;
    cv::Point line, line_2, sqr, sqr_2, sqr_3, sqr_4, center, center_2, erase;
    std::vector<std::string> y_;

    float m2pixel;

    int a = 10;
    int b = 10;
    std::string status = "default";
    void initNode();
    void MapGenerator(std::string dir_path, const std::string &filename, int threshold_occupied, int threshold_free, nav_msgs::OccupancyGrid map);
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
        // cv::Mat img = cv::imread("/home/minji/map_gui/src/Map2/Map1.pgm", 0);
        // cv::Mat img_roi, img_origin;
        //  readMap();
        //   readPGM(&a);
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
    occ_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out", 10);
    // img = cv::imread("/home/minji/map_gui/src/Map2/stMap.pgm", 0);
    // img = cv::imread(file_path, 0);
    ros::Rate loop_rate(10);
    // cv::Mat img = cv::imread("/home/minji/map_gui/src/Map2/Map1.pgm", cv::IMREAD_COLOR);
}

void MODMAP::readMap()
{
    // cv::Mat img = cv::imread("/home/minji/map_gui/src/Map2/stMap.pgm", 0);
    // cv::Mat img = cv::imread(file_path, 0);
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
    std::cout << root << std::endl;
    std::string type = root["type"].asString(); // get_map
    std::string f_ = root["file"].asString();   // get_map
    int width = root["width"].asInt();
    int height = root["height"].asInt();
    int x = root["x"].asInt();
    int y = root["y"].asInt();
    Position = root["cv_pos"];

    // std::vector<std::string> x_split = split(x_, '.');
    // std::vector<std::string> y_split = split(y_, '.');

    // map_x = std::stoi(x); //수정해야함
    // map_y = std::stoi(y);
    // map_width = std::stoi(width);
    // map_height = std::stoi(height);
    float big_size, small_size, resolution, w, h;
    // img = cv::imread("/home/minji/map_gui/src/Map2/Map1.pgm", CV_8UC1);
    img_origin = img.clone();
    img_reset = img.clone();
    cv::Scalar red(0, 0, 255); //지우기
    cv::Scalar green(0, 255, 0);
    cv::Scalar blue(255, 0, 0); //그리기
    std::cout << "ooooooooooooooooooooooooooooooo" << std::endl;
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
    std::cout << "zzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;
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

    std::cout << "why not" << std::endl;
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

    if (type == "map_callback")
    {
        std::cout << "call bvackdf tureu??" << std::endl;
        cv::resize(img_origin, img_origin, cv::Size(w, h));
        mapPublish(img_origin);

        img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
        cv::resize(img_roi, img_roi, cv::Size(400, 400));
        mapBigPublish(img_roi);
    }

    if (type == "plus" || type == "minus")
    {

        int er = 0;
        if (type == "plus")
        {
            plus_cnt++;
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
            minus_cnt++;
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
                roi_res = roi_width / 400;

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

    if (type == "ok_line" || type == "ok_square" || type == "ok_erase")
    {

        if (type == "ok_line")
        {
            center.x = x;
            center.y = y;
            center_2.x = x + roi_width;
            center_2.y = y;

            for (int i = 1; i < 3; i++)
            {
                std::cout << roi_res << std::endl;
                line.x = Position[i][0].asInt() * roi_res;
                std::cout << line.x << std::endl;
                line.x = line.x + x;
                line.y = Position[i][1].asInt() * roi_res;
                line.y = line.y + y;
                pointList_line.push_back(line);
            }

            std::cout << "size : " << pointList_line.size() << std::endl;
            for (int j = 1; j < pointList_line.size() + 1; j++)
            {
                cv::line(color_img, pointList_line[j - 1], pointList_line[j], blue);
                j++;
            }

            for (int i = 0; i < color_img.rows; i++)
            {
                for (int j = 0; j < color_img.cols; j++)
                {
                    if (color_img.at<cv::Vec3b>(i, j)[0] == 255 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 0)
                    {
                        img.at<uchar>(i, j) = 0;
                    }
                    if (color_img.at<cv::Vec3b>(i, j)[0] == 0 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 255)
                    {
                        img.at<uchar>(i, j) = 255;
                    }
                }
            }
        }

        else if (type == "ok_square")
        {
            for (int i = 1; i < 5; i++)
            {
                sqr.x = Position[i][0].asInt() * roi_res;
                sqr.x += x;
                sqr.y = Position[i][1].asInt() * roi_res;
                sqr.y += y;
                pointList_square.push_back(sqr);
            }

            for (int j = 1; j < pointList_square.size() + 1; j++)
            {
                square_j = j;
                if (j % 4 == 0)
                {
                    square_j -= 4;
                }
                cv::line(color_img, pointList_square[j - 1], pointList_square[square_j], blue);
            }

            for (int i = 0; i < color_img.rows; i++)
            {
                for (int j = 0; j < color_img.cols; j++)
                {
                    if (color_img.at<cv::Vec3b>(i, j)[0] == 255 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 0)
                    {
                        img.at<uchar>(i, j) = 0;
                    }
                    if (color_img.at<cv::Vec3b>(i, j)[0] == 0 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 255)
                    {
                        img.at<uchar>(i, j) = 255;
                    }
                }
            }
        }
        else if (type == "ok_erase")
        {
            int erase_np[] = {4};
            cv::Point erase_points[1][4];

            for (int i = 1; i < 5; i++)
            {
                erase.x = Position[i][0].asInt() * roi_res;
                erase.x += x;
                erase.y = Position[i][1].asInt() * roi_res;
                erase.y += y;
                pointList_erase.push_back(erase);
            }
            erase_points[0][0] = pointList_erase[pointList_erase.size() - 4];
            erase_points[0][1] = pointList_erase[pointList_erase.size() - 3];
            erase_points[0][2] = pointList_erase[pointList_erase.size() - 2];
            erase_points[0][3] = pointList_erase[pointList_erase.size() - 1];

            for (int j = 1; j < pointList_erase.size() + 1; j++)
            {
                erase_j = j;
                if (j % 4 == 0)
                {
                    erase_j -= 4;
                }
                cv::line(color_img, pointList_erase[j - 1], pointList_erase[erase_j], red, 1);
            }

            const cv::Point *ppt[1] = {erase_points[0]};
            cv::fillPoly(color_img, ppt, erase_np, 1, red, 8);
            for (int i = 0; i < color_img.rows; i++)
            {
                for (int j = 0; j < color_img.cols; j++)
                {
                    if (color_img.at<cv::Vec3b>(i, j)[0] == 255 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 0)
                    {
                        img.at<uchar>(i, j) = 0;
                    }
                    if (color_img.at<cv::Vec3b>(i, j)[0] == 0 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 255)
                    {
                        img.at<uchar>(i, j) = 255;
                    }
                }
            }
        }

        // cv::imshow("h", img);

        // cv::waitKey(0);
    }

    if (type == "save")
    {

        status = "save";
        //
        std::cout << status << std::endl;
        std::string y_path = "cd /home/minji/map_gui/src/data/CoNA/" + y_[0] + "/; rosrun map_server map_server Map1.yaml";
        system(y_path.c_str());

        std::cout << "path" << std::endl;
        //
    }
    if (type == "file")

    {

        file_path += f_;

        std::cout << file_path << std::endl;
        img = cv::imread(file_path, 0);
        color_img = cv::imread(file_path, 1);

        std::string yaml_path = f_;

        boost::split(y_, f_, boost::is_any_of("/"), boost::algorithm::token_compress_on);
        directory_path = "/home/minji/map_gui/src/data/CoNA/" + y_[0] + "/";
        std::cout << y_[0] << std::endl;
        // std::string y_path = "cd /home/minji/map_gui/src/data/CoNA/" + y_[0] + "/; rosrun map_server map_server Map1.yaml";
        // std::cout << y_path << std::endl;
        // system(y_path.c_str());
    }
}
void MODMAP::mapCallback(nav_msgs::OccupancyGridConstPtr map)
{
    std::cout << "plzddddddddddddddddddddddddddd" << std::endl;
    if (status == "save")
    {
        std::cout << "dddd" << std::endl;

        mtx.lock();
        pgm_occ.header.frame_id = map->header.frame_id;
        pgm_occ.header.seq = map->header.seq;
        pgm_occ.header.stamp = map->header.stamp;

        pgm_occ.info.width = map->info.width;
        pgm_occ.info.height = map->info.height;
        pgm_occ.info.resolution = map->info.resolution;
        pgm_occ.info.map_load_time = map->info.map_load_time;

        pgm_occ.info.origin.position.x = map->info.origin.position.x;
        pgm_occ.info.origin.position.y = map->info.origin.position.y;
        pgm_occ.info.origin.position.z = map->info.origin.position.z;

        pgm_occ.info.origin.orientation.x = map->info.origin.orientation.x;
        pgm_occ.info.origin.orientation.y = map->info.origin.orientation.y;
        pgm_occ.info.origin.orientation.z = map->info.origin.orientation.z;
        pgm_occ.info.origin.orientation.w = map->info.origin.orientation.w;

        // pgm_occ.data.resize(pgm_occ.info.width * pgm_occ.info.height);
        mtx.unlock();
        mtx.lock();
        for (int i = 0; i < map->info.height; i++)
        {
            for (int j = 0; j < map->info.width; j++)
            {
                if (map->data[i * map->info.width + j] == 0)
                {
                    pgm_occ.data.push_back(0);
                }
                else if (map->data[i * map->info.width + j] == 100)
                {
                    pgm_occ.data.push_back(100);
                }
                else
                {
                    pgm_occ.data.push_back(-1);
                }
            }
        }

        mtx.unlock();
        mtx.lock();
        for (int i = 0; i < img.cols; i++)
        {

            for (int j = 0; j < img.rows; j++)
            {
                int j_ = img.cols - j;
                if (color_img.at<cv::Vec3b>(j, i)[0] == 255 && color_img.at<cv::Vec3b>(j, i)[1] == 0 && color_img.at<cv::Vec3b>(j, i)[2] == 0)
                {
                    pgm_occ.data[(img.rows - j - 1) * img.cols + i] = 100;
                }
                else if (color_img.at<cv::Vec3b>(j, i)[0] == 0 && color_img.at<cv::Vec3b>(j, i)[1] == 0 && color_img.at<cv::Vec3b>(j, i)[2] == 255)
                {
                    pgm_occ.data[(img.rows - j - 1) * img.cols + i] = 0;
                }
                //여기에 0쓰면 됨이미지랑 비교
            }
        }

        occ_pub.publish(pgm_occ);
        std::cout << "finish" << std::endl;
        mtx.unlock();
        std::cout << directory_path << std::endl;
        std::cout << filename << std::endl;
        MapGenerator(directory_path, filename, threshold_occupied, threshold_free, pgm_occ);
    }
}

void MODMAP::MapGenerator(std::string dir_path, const std::string &filename_, int threshold_occupied_, int threshold_free_, nav_msgs::OccupancyGrid map)
{
    if (threshold_occupied <= threshold_free)
    {
        std::cout << "threshold_free must be smaller than threshold_occupied" << std::endl;
        return;
    }

    std::string cmd = "mkdir -p " + dir_path;
    int system_return = std::system(cmd.c_str());

    if (system_return != 0)
        printf("system command fail %s", cmd.c_str());
    else
        printf("create directory %s", dir_path.c_str());

    printf("Received a %d X %d map @ %.3f m/pix", map.info.width, map.info.height, map.info.resolution);

    std::string mapdatafile = dir_path + filename_ + ".pgm";
    printf("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE *out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        printf("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            map.info.resolution, map.info.width, map.info.height);
    for (unsigned int y = 0; y < map.info.height; y++)
    {
        for (unsigned int x = 0; x < map.info.width; x++)
        {
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            if (map.data[i] >= 0 && map.data[i] <= threshold_free_)
            { // [0,free)
                fputc(254, out);
            }
            else if (map.data[i] >= threshold_occupied_)
            { // (occ,255]
                fputc(000, out);
            }
            else
            { // occ [0.25,0.65]
                fputc(205, out);
            }
        }
    }

    fclose(out);
}
