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
#include <cmath>
#include <jsoncpp/json/json.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#define TRUE 1
#define FALSE 0
#define PI 3.141592
class MODMAP
{
public:
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    ros::Subscriber sub, sub2, sub3, sub4;
    ros::Publisher pub_pgmsize, occ_pub, file_load;
    image_transport::Publisher map_pub, map_pub_big;
    cv::Mat Mapimage, globalMap, EditedMap, MergedMap, FilteredMap, Map4path;
    cv::Mat img;
    cv::Mat img_roi, img_origin, img_reset;
    std::string file_path = "/home/minji/a/map_gui/src/data/CoNA/";
    // std::string file_path = "/home/cona/data/";
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
    bool minus_switch = true;
    int plus_cnt = 0;
    int minus_cnt = 0;
    std_msgs::String load_msg;

    std::mutex mtx;
    // std::string directory_path = "/home/minji/map_gui/src/data/CoNA/";
    std::string directory_path;
    std::string filename;

    cv::Point line, line_2, sqr, sqr_2, sqr_3, sqr_4, center, center_2, erase, square;
    std::vector<std::string> y_;
    std::vector<std::string> yaml;
    std::vector<std::string> origin_value;
    std::vector<std::string> name_parsing;
    std::vector<std::string> txt_value;

    std::vector<float> txt_value_x;
    std::vector<float> txt_value_y;

    std::vector<int> txt_point_x;
    std::vector<int> txt_point_y;

    // 회전 전 화살표 포인트 좌표

    std::vector<int> txt_pointer_x;
    std::vector<int> txt_pointer_y;

    std::vector<int> trans_point_x;
    std::vector<int> trans_point_y;

    std::vector<int> trans_Lpoint_x;
    std::vector<int> trans_Lpoint_y;

    std::vector<int> trans_Rpoint_x;
    std::vector<int> trans_Rpoint_y;

    // (0, 0) 기준으로 해서 회전 전 좌표
    std::vector<int> zero_x;
    std::vector<int> zero_y;

    // 기준좌표로부터 거리(y축 대칭 시켜줄거라 ㅎ)
    std::vector<int> distance;

    std::vector<int> Ldistance;
    std::vector<int> Rdistance;

    float m2pixel;

    float origin_x;
    float origin_y;

    float origin_to_mat_x;
    float origin_to_mat_y;

    int a = 10;
    int b = 10;
    std::string status = "default";
    void initNode();
    void MapGenerator(std::string dir_path, const std::string &filename, int threshold_occupied, int threshold_free, nav_msgs::OccupancyGrid map);
    int roi_x, roi_y, roi_height;
    float roi_width;
    // void mapCallback(nav_msgs::OccupancyGridConstPtr map);
    void readMap();
    // void mapViewCallback()
    void btnCallback(const std_msgs::String::ConstPtr &msg);
    void jsonCallback(const std_msgs::String::ConstPtr &msg);
    void mapPublish(cv::Mat image);
    void mapBigPublish(cv::Mat image);
    void mouseCallback(int event, int x, int y, int flags, void *userdata);

    MODMAP()
        : it(n)
    {
        initNode();
    }
};

void MODMAP::initNode()
{
    // sub = n.subscribe("/map", 1, &MODMAP::mapCallback, this);
    sub2 = n.subscribe("/btnInput", 1, &MODMAP::btnCallback, this);
    sub3 = n.subscribe("/btnInput", 1, &MODMAP::jsonCallback, this);
    sub4 = n.subscribe("/mappos", 1, &MODMAP::jsonCallback, this);
    map_pub = it.advertise("map_pgm", 1);
    map_pub_big = it.advertise("map_big", 1);
    file_load = n.advertise<std_msgs::String>("file_load", 1);
    // occ_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out", 10);
    //  img = cv::imread("/home/minji/map_gui/src/Map2/stMap.pgm", 0);
    //  img = cv::imread(file_path, 0);
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

void MODMAP::mouseCallback(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
        std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        std::cout << "Left button of the mouse is released - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_RBUTTONUP)
    {
        std::cout << "Right button of the mouse is released - position (" << x << ", " << y << ")" << std::endl;
    }
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

    std::string local_file_path = file_path + f_;

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

    if (type == "map_draw")
    {
        int er = 0;
        a = 10;
        b = 10;

        try
        {
            er = 5;
            if (x <= 0 || y <= 0 || x + 40 > img.cols || y + 40 > img.rows)
            {
                throw er;
            }
            img_roi = img(cv::Rect(x, y, 40, 40));
            cv::resize(img_roi, img_roi, cv::Size(400, 400));
            std::cout << img_roi.channels() << std::endl;
            roi_x = x;
            roi_y = y;
            roi_height = 40;
            roi_width = 40;

            mapBigPublish(img_roi);
        }
        catch (int er)
        {
            std::cout << er << std::endl;
        }
    }

    else if (type == "plus" || type == "minus")
    {

        int er = 0;
        if (type == "plus" && minus_switch == true)
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
                if (roi_height < 40 || roi_width < 40 || roi_x <= 0 || roi_y <= 0 || roi_x + roi_width >= img.cols || roi_y + roi_height >= img.rows)
                {
                    minus_switch = false;
                    std::cout << "bbbbb" << std::endl;
                    throw er;
                }
                img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                cv::resize(img_roi, img_roi, cv::Size(400, 400));
                // cv::cvtColor(img_roi, gray, CV_GRAY2RGB);

                mapBigPublish(img_roi);
                // a += 10;

                std::cout << roi_x << std::endl;
            }
            catch (int er)
            {
                std::cout << er << std::endl;
            }
        }

        else if (type == "minus")
        {
            minus_cnt++;
            try
            {

                std::cout << "minus" << std::endl;
                roi_x = roi_x - 10;
                roi_y = roi_y - 10;
                roi_height = roi_height + 2 * b;
                roi_width = roi_width + 2 * b;
                a = 10;
                er = 9;
                if (minus_switch == false)
                {
                    minus_switch = true;
                }
                if (roi_x <= 0 || roi_y <= 0 || roi_x + roi_width >= img.cols || roi_y + roi_height >= img.rows)
                {
                    std::cout << "bbbbb" << std::endl;
                    throw er;
                }
                img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                cv::resize(img_roi, img_roi, cv::Size(400, 400));

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

    else if (type == "up" || type == "down" || type == "right" || type == "left")
    {

        roi_x = x;
        roi_y = y;

        if (roi_x >= 0 && roi_x + roi_width <= img.cols && roi_y >= 0 && roi_y + roi_height <= img.rows)
        {
            std::cout << "roi_w : " << roi_width << std::endl;
            std::cout << "roi_h : " << roi_height << std::endl;
            img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
            cv::resize(img_roi, img_roi, cv::Size(400, 400));
            mapBigPublish(img_roi);
        }
    }

    if (type == "ok_line" || type == "ok_square" || type == "ok_erase")
    {
        std::vector<cv::Point> pointList_line; /// vector좌표 push해서 좌표가지고 사각형 그리고, +, - 구현하기
        std::vector<cv::Point> pointList_square;
        std::vector<cv::Point> pointList_erase;
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
                }
            }
        }

        else if (type == "ok_square")
        {
            int square_np[] = {4};
            cv::Point square_points[1][4];

            for (int i = 1; i < 5; i++)
            {
                square.x = Position[i][0].asInt() * roi_res;
                square.x += x;
                square.y = Position[i][1].asInt() * roi_res;
                square.y += y;
                pointList_square.push_back(square);
            }

            square_points[0][0] = pointList_square[pointList_square.size() - 4];
            square_points[0][1] = pointList_square[pointList_square.size() - 3];
            square_points[0][2] = pointList_square[pointList_square.size() - 2];
            square_points[0][3] = pointList_square[pointList_square.size() - 1];

            for (int j = 1; j < pointList_square.size() + 1; j++)
            {
                square_j = j;
                if (j % 4 == 0)
                {
                    square_j -= 4;
                }
                cv::line(color_img, pointList_square[j - 1], pointList_square[square_j], blue, 1);
            }

            const cv::Point *sppt[1] = {square_points[0]};
            cv::fillPoly(color_img, sppt, square_np, 1, blue, 8);
            for (int i = 0; i < color_img.rows; i++)
            {
                for (int j = 0; j < color_img.cols; j++)
                {

                    if (color_img.at<cv::Vec3b>(i, j)[0] == 255 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 0)
                    {
                        img.at<uchar>(i, j) = 0;
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

                    if (color_img.at<cv::Vec3b>(i, j)[0] == 0 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 255)
                    {
                        img.at<uchar>(i, j) = 255;
                    }
                }
            }
        }

        std::cout << "call bvackdf tureu??" << std::endl;
        cv::resize(img, img_origin, cv::Size(w, h));
        mapPublish(img_origin);

        img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
        cv::resize(img_roi, img_roi, cv::Size(400, 400));
        mapBigPublish(img_roi);

        // cv::imshow("h", img);

        // cv::waitKey(0);
    }

    else if (type == "save")
    {

        status = "save";

        std::cout << "fssssssssssssssssssssssssss" << std::endl;
        // std::cout << status << std::endl;
        // std::string y_path = "cd /home/minji/map_gui/src/data/CoNA/" + y_[0] + "/; rosrun map_server map_server Map1.yaml";
        // system(y_path.c_str());

        // std::cout << "path" << std::endl;

        pgm_occ.info.width = img.cols;
        pgm_occ.info.height = img.rows;
        pgm_occ.info.resolution = 0.025000;

        for (int i = 0; i < img.rows; i++)
        {

            for (int j = 0; j < img.cols; j++)
            {

                if (color_img.at<cv::Vec3b>(i, j)[0] == 255 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 0)
                {
                    pgm_occ.data[img.cols * i + j] = 100;
                }

                else if (color_img.at<cv::Vec3b>(i, j)[0] == 0 && color_img.at<cv::Vec3b>(i, j)[1] == 0 && color_img.at<cv::Vec3b>(i, j)[2] == 255)
                {
                    pgm_occ.data[img.cols * i + j] = 0;
                }
            }
        }

        // occ_pub.publish(pgm_occ);
        std::cout << "finish" << std::endl;

        std::cout << directory_path << std::endl;
        std::cout << filename << std::endl;
        MapGenerator(directory_path, filename, threshold_occupied, threshold_free, pgm_occ);
        //
    }
    else if (type == "path")
    {

        std::vector<std::string> txt_place;
        std::vector<float> angle_;

        std::cout << "ppppppppppppppppath" << std::endl;
        std::ifstream readFile;
        readFile.open("/home/minji/a/map_gui/src/data/CoNA/Map2/Map1.yaml");
        if (readFile.is_open())
        {
            while (!readFile.eof())
            {
                std::string str;
                std::getline(readFile, str);
                boost::split(yaml, str, boost::is_any_of("["), boost::algorithm::token_compress_on);
                for (int i = 0; i < yaml.size(); i++)
                {
                    // if(i % 2 == 0)
                    //{
                    if (yaml[i] == "origin: ")
                    {
                        std::cout << yaml[i + 1] << std::endl;
                        boost::split(origin_value, yaml[i + 1], boost::is_any_of(","), boost::algorithm::token_compress_on);
                        std::cout << origin_value[0] << std::endl;
                        std::cout << origin_value[1] << std::endl;

                        origin_x = -1 * stof(origin_value[0]);

                        origin_y = -1 * stof(origin_value[1]);

                        std::cout << origin_x << std::endl;
                        std::cout << origin_y << std::endl;
                    }
                }
            }

            float map_resolution = 0.025;
            origin_to_mat_x = int(origin_x / map_resolution);
            origin_to_mat_y = color_img.rows - int(origin_y / map_resolution);

            std::cout << origin_to_mat_x << std::endl;
            std::cout << origin_to_mat_y << std::endl;
            readFile.close();

            readFile.open("/home/minji/a/map_gui/src/data/CoNA/Map2/map_file.txt");
            if (readFile.is_open())
            {
                while (!readFile.eof())
                {
                    std::string str_txt;
                    std::getline(readFile, str_txt);
                    boost::split(txt_value, str_txt, boost::is_any_of(","), boost::algorithm::token_compress_on);

                    for (int i = 0; i < txt_value.size(); i++)
                    {
                        if (i % 6 == 1)
                        {
                            txt_value_x.push_back(stof(txt_value[i]));
                        }
                        else if (i % 6 == 2)
                        {
                            txt_value_y.push_back(stof(txt_value[i]));
                        }
                        if (i % 6 == 3)
                        {

                            if (stof(txt_value[i]) < 0)
                            {
                                angle_.push_back(360 + stof(txt_value[i]));
                            }
                            else
                            {
                                angle_.push_back(stof(txt_value[i]));
                            }
                        }
                        if (i % 6 == 4)
                        {
                            txt_place.push_back(txt_value[i]);
                        }
                    }
                }
            }

            for (int i = 0; i < txt_value_x.size(); i++)
            {
                int distance_val;

                // int distance_Lval;
                // int distance_Rval;

                std::cout << i << "  angle : " << angle_[i] << std::endl;
                txt_point_x.push_back(origin_to_mat_x + txt_value_x[i] / 0.025);
                txt_point_y.push_back(origin_to_mat_y - txt_value_y[i] / 0.025);

                txt_pointer_x.push_back(txt_point_x[i] + 9);
                txt_pointer_y.push_back(txt_point_y[i]);

                zero_x.push_back(std::cos(angle_[i] * PI / 180) * 9 - std::sin(angle_[i] * PI / 180) * 0 + txt_point_x[i]);
                zero_y.push_back(std::sin(angle_[i] * PI / 180) * 9 + std::cos(angle_[i] * PI / 180) * 0 + txt_point_y[i]);

                trans_Lpoint_x.push_back(std::cos(angle_[i] * PI / 180) * -5 - std::sin(angle_[i] * PI / 180) * 5 + txt_point_x[i]);
                trans_Lpoint_y.push_back(std::sin(angle_[i] * PI / 180) * -5 + std::cos(angle_[i] * PI / 180) * 5 + txt_point_y[i]);

                trans_Rpoint_x.push_back(std::cos(angle_[i] * PI / 180) * -5 - std::sin(angle_[i] * PI / 180) * -5 + txt_point_x[i]);
                trans_Rpoint_y.push_back(std::sin(angle_[i] * PI / 180) * -5 + std::cos(angle_[i] * PI / 180) * -5 + txt_point_y[i]);

                distance_val = zero_x[i] - txt_point_x[i];

                if (distance_val < 0)
                {
                    distance_val *= -1;
                    distance.push_back(txt_point_x[i] + distance_val);
                }
                else
                {
                    distance.push_back(txt_point_x[i] - distance_val);
                }

                // distance_Lval = trans_Lpoint_x[i] - txt_point_x[i];

                // if (distance_Lval < 0)
                // {
                //     distance_Lval *= -1;
                //     Ldistance.push_back(txt_point_x[i] + distance_Lval);
                // }
                // else
                // {
                //     Ldistance.push_back(txt_point_x[i] - distance_Lval);
                // }

                // distance_Rval = trans_Rpoint_x[i] - txt_point_x[i];

                // if (distance_Rval < 0)
                // {
                //     distance_Rval *= -1;
                //     Rdistance.push_back(txt_point_x[i] + distance_Rval);
                // }
                // else
                // {
                //     Rdistance.push_back(txt_point_x[i] - distance_Rval);
                // }

                std::string a = std::to_string(i);

                std::cout << i << "     " << txt_point_x[i] << "      " << txt_point_y[i] << std::endl;
                // std::cout << i+1 << "  x : " << txt_value_x[i] << "  y : " << txt_value_y[i] << std::endl;

                // cv::Point trian_[3] = {{Ldistance[i], trans_Lpoint_y[i]}, {txt_point_x[i], txt_point_y[i]}, {Rdistance[i], trans_Rpoint_y[i]}};
                // cv::Point *t[1] = {trian_};
                // int tri_npts[1] = {3};
                // cv::polylines(color_img, t, tri_npts, 1, 0, blue);

                // cv::circle(color_img, cv::Point(txt_point_x[i], txt_point_y[i]), 2, green, -1);
                cv::arrowedLine(color_img, cv::Point(txt_point_x[i], txt_point_y[i]), cv::Point(distance[i], zero_y[i]), blue, 1);

                //글자나오게하는 곳
                if (txt_place[i] != " none")
                {
                    cv::putText(color_img, txt_place[i], cv::Point(txt_point_x[i] + 7, txt_point_y[i] - 4), 2, 0.4, red);
                }
                // cv::putText(color_img, a, cv::Point(txt_point_x[i] + 2, txt_point_y[i] + 2), 2, 0.4, red);
            }
            readFile.close();
        }
        cv::circle(color_img, cv::Point(origin_to_mat_x, origin_to_mat_y), 5, green, -1);
        // cv::arrowedLine(color_img, cv::Point(origin_to_mat_x, origin_to_mat_y), cv::Point(origin_to_mat_x + 10, origin_to_mat_y), green);
        cv::namedWindow("circle", 1);
        cv::setMouseCallback("circle", MODMAP::mouseCallback, this);
        cv::imshow("circle", color_img);
        cv::waitKey(0);
    }

    else if (type == "file")

    {
        std::stringstream ss;
        // file_path += f_;
        ss.str("");
        std::cout << file_path << std::endl;
        std::cout << local_file_path << std::endl;
        img = cv::imread(local_file_path, 0);
        color_img = cv::imread(local_file_path, 1);

        img_origin = img.clone();
        img_reset = img.clone();

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

        if (!img.empty())
        {
            ss << "success";
            std::string yaml_path = f_;

            // boost::split(y_, f_, boost::is_any_of("/"), boost::algorithm::token_compress_on);
            // boost::split(name_parsing, y_[2], boost::is_any_of("."), boost::algorithm::token_compress_on);
            //  directory_path = "/home/minji/map_gui/src/data/CoNA/" + y_[0] + "/";
            // directory_path = "/home/cona/data/" + y_[0] + "/" + y_[1] + "/";
            directory_path = "/home/minji/a/map_gui/src/data/CoNA/";
            // std::cout << y_[0] << std::endl;
            // std::cout << y_[1] << std::endl;
            // std::cout << y_[2] << std::endl;
            // filename = name_parsing[0];
            // std::cout << name_parsing[0] << std::endl;

            // std::string y_path = "cd /home/minji/map_gui/src/data/CoNA/" + y_[0] + "/; rosrun map_server map_server Map1.yaml";
            // std::cout << y_path << std::endl;
            // system(y_path.c_str());

            for (int i = 0; i < img.rows; i++)
            {

                for (int j = 0; j < img.cols; j++)
                {
                    if (img.at<uchar>(i, j) <= 110)
                    {
                        pgm_occ.data.push_back(100);
                    }

                    else if (img.at<uchar>(i, j) <= 220)
                    {
                        pgm_occ.data.push_back(-1);
                    }

                    else
                    {
                        pgm_occ.data.push_back(0);
                    }
                }
            }
        }
        else
        {
            std::cout << "image empty !! " << std::endl;
            // file_path = "/home/cona/data/";
            file_path = "/home/minji/a/map_gui/src/data/CoNA/";
            ss << "fail";
        }
        // load_msg.data = ss.str();
        // file_load.publish(load_msg);

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
            // unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            unsigned int i = y * map.info.width + x;
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
