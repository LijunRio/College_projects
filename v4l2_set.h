#pragma once
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <iostream>
class v4l2_set
{
public:
    void vel2_camera(const char* file_name,int exposure);
    void v4l2_saturation(const char* file_name,int saturation);
    void Get_v4l2_saturation(const char* file_name);
};
