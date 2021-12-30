#include "ros/ros.h"
#include "std_msgs/String.h"

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


#define TRUE	1
#define FALSE	0

class MODMAP
{
    public:

        typedef struct {
            char M, N;
            int width;
            int height;
            int max;
            unsigned char **pixels;
        } PGMImage;

        ros::NodeHandle n;
        ros::Publisher chatter_pub;
        void initNode();
        PGMImage a;
        int readPGM(PGMImage *img);
        void closePGM(PGMImage *img);
        char b[10] = "Map1.pgm";

        MODMAP()
        {
            initNode();
            readPGM(&a);
        }
};

void MODMAP::initNode()
{
    chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);
    
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        //ros::spinOnce();

        loop_rate.sleep();
        ++count;
        if (count == 10){
            break;
        }
    }
}

int MODMAP::readPGM(PGMImage *img){
    ros::Rate loop_rate(10);
    FILE* fp = fopen("/home/cona/map_gui/src/map/Map1.pgm", "r");
    if(fp == NULL){
		fprintf(stderr, "파일을 열 수 없습니다 : %s\n", "Map1.pgm");
		return FALSE;
	}
    fscanf(fp, "%c %c", &img->M    , &img->N     );   // 매직넘버 읽기
    if(img->M != 'P' || img->N != '5'){
		fprintf(stderr, "PGM 이미지 포멧이 아닙니다 : %c%c\n", img->M, img->N);
		return FALSE;
	}

    fscanf(fp, "%d %d", &img->width, &img->height);   // 가로, 세로 읽기
    fscanf(fp, "%d"   , &img->max                );	// 최대명암도 값

	if(img->max != 255){
		fprintf(stderr, "올바른 이미지 포멧이 아닙니다.\n");
        return FALSE;
    }
 
    // <-- 메모리 할당
    img->pixels = (unsigned char**)calloc(img->height, sizeof(unsigned char*));
 
    for(int i=0; i<img->height; i++){
        img->pixels[i] = (unsigned char*)calloc(img->width, sizeof(unsigned char));
    }
    // -->
 
    // <-- pbm 파일로부터 픽셀값을 읽어서 할당한 메모리에 load
    int tmp;
    for(int i=0; i<img->height; i++){
        for(int j=0; j<img->width; j++){
            fscanf(fp, "%d", &tmp);
            if (tmp != 0){
                printf((const char*)tmp);
            }
            std::cout << tmp << std::endl;
            img->pixels[i][j] = (unsigned char)tmp;
        }
    }
    

    fclose(fp); // 더 이상 사용하지 않는 파일을 닫아 줌
    
    return TRUE;
}

void MODMAP::closePGM(PGMImage* img)
{
	for(int i=0; i<img->height; i++){
		free(img->pixels[i]);
	}

	free(img->pixels);
}

