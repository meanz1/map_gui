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

//static int me_flag = 0;
//static std::vector<cv::Point> line_path;

class MODMAP
{
    public:
        ros::NodeHandle n;
        image_transport::ImageTransport it;
        ros::Subscriber sub, sub2, sub3, sub4;
        ros::Publisher pub_pgmsize, occ_pub, file_load;
        image_transport::Publisher map_pub, map_pub_big, map_pub_path;
        cv::Mat Mapimage, globalMap, EditedMap, MergedMap, FilteredMap, Map4path;
        cv::Mat img, color_img;
        cv::Mat img_roi, img_origin, img_reset;
        cv::Mat path_img, path_img_roi;
        cv::Mat path_cp_img;
        std::string file_path = "/home/minji/a/map_gui/src/data/CoNA/";
      
        nav_msgs::OccupancyGrid pgm_occ;

        int map_width, map_height;
        int map_x, map_y;
        int square_j, erase_j;
        float roi_res;
        int threshold_occupied = 65;
        int threshold_free = 25;
        bool minus_switch = true;
        bool path_flag = false;
        int plus_cnt = 0;
        int minus_cnt = 0;
        std_msgs::String load_msg;

        int mini_can_w;
        int mini_can_h;

        std::mutex mtx;
       
        std::string directory_path;
        std::string filename;

        cv::Point line, line_2, sqr, sqr_2, sqr_3, sqr_4, center, center_2, erase, square, arrow, arrow_2;
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

        // (0, 0) 기준으로 해서 회전 전 좌표
        std::vector<int> zero_x;
        std::vector<int> zero_y;

        // 기준좌표로부터 거리(y축 대칭 시켜줄거라 ㅎ)
        std::vector<int> distance;

        std::vector<int> Ldistance;
        std::vector<int> Rdistance;

        std::vector<std::string> txt_place;
        std::vector<float> angle_;

        float m2pixel;

        double origin_x;
        double origin_y;

        float origin_to_mat_x;
        float origin_to_mat_y;

        int a = 10;
        int b = 10;
        int roi_x, roi_y, roi_height;
        float roi_width;
        double map_resolution = 0.02500;

        int file_count_n = 33;

        float changedAngle;

        std::string status = "default";

        void initNode();
        void MapGenerator(std::string dir_path, const std::string &filename, int threshold_occupied, int threshold_free, nav_msgs::OccupancyGrid map);
        
        void btnCallback(const std_msgs::String::ConstPtr &msg);
        void jsonCallback(const std_msgs::String::ConstPtr &msg);
        
        void mapPublish(cv::Mat image);
        void mapBigPublish(cv::Mat image);

        void mat2txt();
       

        MODMAP()
            : it(n)
        {
            initNode();
        }
    
};

void MODMAP::initNode()
{
    sub3 = n.subscribe("/btnInput", 1, &MODMAP::jsonCallback, this);
    sub4 = n.subscribe("/mappos", 1, &MODMAP::jsonCallback, this);
    map_pub = it.advertise("map_pgm", 1);
    map_pub_big = it.advertise("map_big", 1);
    file_load = n.advertise<std_msgs::String>("file_load", 1);
    pub_pgmsize = n.advertise<std_msgs::Float32MultiArray>("mapsize", 100);
    ros::Rate loop_rate(10);
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

    std::string local_file_path = file_path + f_;

    int width = root["width"].asInt();
    int height = root["height"].asInt();
    int x = root["x"].asInt();
    int y = root["y"].asInt();
    Position = root["cv_pos"];

    float big_size, small_size, resolution, w, h;
    
    img_origin = img.clone();
    img_reset = img.clone();
    cv::Scalar red(0, 0, 255); //지우기
    cv::Scalar green(0, 255, 0);
    cv::Scalar blue(255, 0, 0); //그리기

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
    std_msgs::Float32MultiArray mapsize;

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
            
            roi_x = x;
            roi_y = y;
            roi_height = 40;
            roi_width = 40;
            
            if (path_flag == true) 
            {
                std::cout<<"map_draw -> path_flag : true" << std::endl;
                path_img_roi = path_img(cv::Rect(x, y, 40, 40));
                cv::resize(path_img_roi, path_img_roi, cv::Size(400, 400));
                
                mapBigPublish(path_img_roi);
            }
            else 
            {
                img_roi = img(cv::Rect(x, y, 40, 40));
                cv::resize(img_roi, img_roi, cv::Size(400, 400));
                std::cout << img_roi.channels() << std::endl;
                mapBigPublish(img_roi);
            }
            
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

                if(path_flag == true)
                {
                    path_img_roi = path_img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                    cv::resize(path_img_roi, path_img_roi, cv::Size(400, 400));
                    mapBigPublish(path_img_roi);
                }

                else
                {
                    img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                    cv::resize(img_roi, img_roi, cv::Size(400, 400));
                    mapBigPublish(img_roi);
                }
               
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
                    throw er;
                }
                
                roi_res = roi_width / 400;

                if(path_flag == true)
                {
                    path_img_roi = path_img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                    cv::resize(path_img_roi, path_img_roi, cv::Size(400, 400));
                    mapBigPublish(path_img_roi);
                }

                else
                {
                    img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                    cv::resize(img_roi, img_roi, cv::Size(400, 400));
                    mapBigPublish(img_roi);
                }
                
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
            if(path_flag == true)
            {
                path_img_roi = path_img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                cv::resize(path_img_roi, path_img_roi, cv::Size(400, 400));
                mapBigPublish(path_img_roi);
            }
            else
            {
                img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
                cv::resize(img_roi, img_roi, cv::Size(400, 400));
                mapBigPublish(img_roi);
            }
        }
    }

    if (type == "ok_arrow")
    {
        std::vector<cv::Point> pointList_arrow;
      
        if (type == "ok_arrow")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        {
            double line_length;
            float gradient;
            for (int i = 1; i < 3; i++)
            {
                std::cout << roi_res << std::endl;
                arrow.x = Position[i][0].asInt() * roi_res;
                std::cout << arrow.x << std::endl;
                arrow.x = arrow.x + x;
                arrow.y = Position[i][1].asInt() * roi_res;
                arrow.y = arrow.y + y;
                pointList_arrow.push_back(arrow);
            }

            line_length = sqrt(pow(pointList_arrow[0].x - pointList_arrow[1].x, 2) + pow(pointList_arrow[0].y - pointList_arrow[1].y, 2));
            
            std::cout << "line length : " << line_length << std::endl;
            std::cout << "n = " << line_length/8 << std::endl;

            // 선 긋기
            // for (int i = 0; i <= line_length/4; i ++)
            // {
            //     cv::line(path_img, cv::Point(pointList_arrow[0].x + (int)((pointList_arrow[1].x-pointList_arrow[0].x)/(line_length/4)*i), pointList_arrow[0].y + (int)((pointList_arrow[1].y-pointList_arrow[0].y)/(line_length/4)*i)), cv::Point(pointList_arrow[0].x + (int)((pointList_arrow[1].x-pointList_arrow[0].x)/(line_length/4)*i), pointList_arrow[0].y + (int)((pointList_arrow[1].y-pointList_arrow[0].y)/(line_length/4)*i)+10),green);
            // }

            for (int i = 1; i <= line_length/8; i ++)
            {
                cv::arrowedLine(path_img, cv::Point(pointList_arrow[0].x + (int)((pointList_arrow[1].x-pointList_arrow[0].x)/(line_length/8)*(i-1)), pointList_arrow[0].y + (int)((pointList_arrow[1].y-pointList_arrow[0].y)/(line_length/8)*(i-1))), cv::Point(pointList_arrow[0].x + (int)((pointList_arrow[1].x-pointList_arrow[0].x)/(line_length/8)*i), pointList_arrow[0].y + (int)((pointList_arrow[1].y-pointList_arrow[0].y)/(line_length/8)*i)), green, 1);
                i++;
            }
            // map_file.txt에 들어갈 각도.
            changedAngle = -atan2(pointList_arrow[1].y - pointList_arrow[0].y, pointList_arrow[1].x - pointList_arrow[0].x)*180/PI;
            
            std::fstream fs_file("/home/minji/a/map_gui/src/data/CoNA/Map2/map_file.txt");
    
            // click coordination changes to map_file.txt coordination 
            if (fs_file.is_open())
            {
                file_count_n += 1;
                std::cout<< "file_count_n : " << file_count_n << std::endl;
                fs_file.seekg(-4, std::ios::end);
                fs_file << file_count_n << ", " << (pointList_arrow[0].x - origin_x/map_resolution)*map_resolution + 0.02 << ", " << (color_img.rows - pointList_arrow[0].y - origin_y/map_resolution)*map_resolution << ", " << changedAngle << ", none, -1;" << std::endl;
                for (int i = 1; i < line_length/8; i++)
                {
                    // std::cout << (txt_point_x[i] - origin_x/map_resolution)*map_resolution + 0.02 << std::endl;
                    // std::cout << (color_img.rows - txt_point_y[i] - origin_y/map_resolution)*map_resolution << std::endl;
                    file_count_n ++;
                    fs_file << file_count_n << ", " << ((pointList_arrow[0].x + (int)((pointList_arrow[1].x-pointList_arrow[0].x)/(line_length/8)*i)) - origin_x/map_resolution)*map_resolution + 0.02 << ", " << (color_img.rows - (pointList_arrow[0].y + (int)((pointList_arrow[1].y-pointList_arrow[0].y)/(line_length/8)*i)) - origin_y/map_resolution)*map_resolution << ", " << changedAngle << ", none, -1;" << std::endl;
                }
            }

            fs_file << file_count_n << ", " << (pointList_arrow[1].x - origin_x/map_resolution)*map_resolution + 0.02 << ", " << (color_img.rows - pointList_arrow[1].y - origin_y/map_resolution)*map_resolution << ", " << changedAngle << ", none, -1;" << std::endl;
            fs_file <<"end"<<std::endl;

            fs_file.clear();
            fs_file.close();
        }

        if (path_flag == true)
        {
            std::cout << "when path_flag is true, " << std::endl;
            cv::resize(path_img, path_cp_img, cv::Size(w, h));
            mapPublish(path_cp_img);

            path_img_roi = path_img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
            cv::resize(path_img_roi, path_img_roi, cv::Size(400, 400));
            mapBigPublish(path_img_roi);
        }

        else 
        {
            std::cout << "call bvackdf tureu??" << std::endl;
            cv::resize(img, img_origin, cv::Size(w, h));
            mapPublish(img_origin);

            img_roi = img(cv::Rect(roi_x, roi_y, roi_width, roi_height));
            cv::resize(img_roi, img_roi, cv::Size(400, 400));
            mapBigPublish(img_roi);
        }
       
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
        
        // path 띄울 매트릭스 하나 만듦
        path_img = cv::imread(local_file_path, 0);
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

        mini_can_h = h;
        mini_can_w = w;

        mapsize.data.push_back(w);
        mapsize.data.push_back(h);
        mapsize.data.push_back(resolution);
        pub_pgmsize.publish(mapsize);

        path_flag = true;

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

            origin_to_mat_x = int(origin_x / map_resolution);
            origin_to_mat_y = color_img.rows - int(origin_y / map_resolution);

            std::cout << origin_to_mat_x << std::endl;
            std::cout << origin_to_mat_y << std::endl;
            readFile.close();

            readFile.open("/home/minji/a/map_gui/src/data/CoNA/Map2/map_file.txt");
            std::cout << "1" << std::endl;
            if (readFile.is_open())
            {
                while (!readFile.eof())
                {
                    std::string str_txt;
                    std::getline(readFile, str_txt);
                    boost::split(txt_value, str_txt, boost::is_any_of(","), boost::algorithm::token_compress_on);

                    for (int i = 0; i < txt_value.size()-1; i++)
                    {
                        if (i % 6 == 0)
                        {
                            file_count_n = stoi(txt_value[i]);
                        }
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

                txt_point_x.push_back(origin_to_mat_x + txt_value_x[i] / map_resolution);
                txt_point_y.push_back(origin_to_mat_y - txt_value_y[i] / map_resolution);

                zero_x.push_back(std::cos(angle_[i] * PI / 180) * 9 - std::sin(angle_[i] * PI / 180) * 0 + txt_point_x[i]);
                zero_y.push_back(std::sin(angle_[i] * PI / 180) * 9 + std::cos(angle_[i] * PI / 180) * 0 + txt_point_y[i]);

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

                std::string a = std::to_string(i);

                cv::arrowedLine(path_img, cv::Point(txt_point_x[i], txt_point_y[i]), cv::Point(distance[i], zero_y[i]), green, 1);

                //글자나오게하는 곳
                if (txt_place[i] != " none")
                {
                    cv::putText(path_img, txt_place[i], cv::Point(txt_point_x[i] + 7, txt_point_y[i] - 4), 2, 0.4, red);
                }
                // cv::putText(color_img, a, cv::Point(txt_point_x[i] + 2, txt_point_y[i] + 2), 2, 0.4, red);
            }
            readFile.close();
        }
        cv::circle(path_img, cv::Point(origin_to_mat_x, origin_to_mat_y), 5, green, -1);

        //path_cp_img = path_img(cv::Rect(0, 0, path_img.cols, path_img.rows));
        cv::resize(path_img, path_cp_img, cv::Size(mini_can_w, mini_can_h));
        // cv::arrowedLine(color_img, cv::Point(origin_to_mat_x, origin_to_mat_y), cv::Point(origin_to_mat_x + 10, origin_to_mat_y), green);
        mapPublish(path_cp_img);
    }
}