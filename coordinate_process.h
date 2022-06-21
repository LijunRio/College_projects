#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include "serialport.h"

class coordinate_process
{
public:
    void CoorProcess_Send(cv::Point final_send,SerialPort port,int color_flag);
    void a_send(cv::Point final_send,SerialPort port);
};
