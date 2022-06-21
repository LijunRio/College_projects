#include "coordinate_process.h"
#include"serialport.h"
#include<iostream>
using namespace std;
using namespace cv;

//最后得到的坐标根据通信协议进行处理
//X--Y--Color_flag
void coordinate_process::CoorProcess_Send(cv::Point final_send,SerialPort port,int color_flag)
{
    string point_x_str;
    std::stringstream point_x;
    point_x<<final_send.x;
    point_x>>point_x_str;

    string point_y_str;
    std::stringstream point_y;
    point_y<<final_send.y;
    point_y>>point_y_str;

    if(color_flag==1)
    {
        string coordinate;
        coordinate = "X"+point_x_str + "Y" +point_y_str + "E";
        const char* coordinate_red = coordinate.c_str();
        char* coordinate_send = new char[9];
        std::strcpy(coordinate_send,coordinate_red);
        port.Send(coordinate_send);

    }
    else if(color_flag == 2)
    {
        string coordinate;
        coordinate = "X"+point_x_str + "Y" +point_y_str + "E";
        const char* coordinate_blue = coordinate.c_str();
        char* coordinate_send = new char[9];
        std::strcpy(coordinate_send,coordinate_blue);
        port.Send(coordinate_send);

    }

}

//考虑到机械电控的延时，增加了一个提前打子弹的标志位。
//如果目标在瞄准点附近就发标志位A
 void coordinate_process:: a_send(cv::Point final_send,SerialPort port)
{
     string point_x_str;
     std::stringstream point_x;
     point_x<<final_send.x;
     point_x>>point_x_str;

     string point_y_str;
     std::stringstream point_y;
     point_y<<final_send.y;
     point_y>>point_y_str;

     string coordinate;
     coordinate = "A"+point_x_str + "Y" +point_y_str + "E";
     const char* coordinate_red = coordinate.c_str();
     char* coordinate_send = new char[9];
     std::strcpy(coordinate_send,coordinate_red);
     port.Send(coordinate_send);
}
