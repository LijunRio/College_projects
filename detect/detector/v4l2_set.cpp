#include<iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include"detector_methods.h"
#include"coordinate_process.h"
#include<time.h>
#include<QDebug>
#include<QTime>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include<v4l2_set.h>
using namespace std;
using namespace cv;
//设置曝光
void v4l2_set::vel2_camera(const char* file_name,int exposure)
{
    int Handle0=open(file_name,O_RDWR);
    struct v4l2_control ctrl0;
    ctrl0.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ioctl(Handle0,VIDIOC_S_CTRL,&ctrl0);
    cout<<ctrl0.value<<endl;

    ctrl0.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl0.value = 1;
    ioctl(Handle0,VIDIOC_S_CTRL,&ctrl0);
    cout<<ctrl0.value<<endl;

    ctrl0.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl0.value = exposure;

    ioctl(Handle0,VIDIOC_S_CTRL,&ctrl0);
    cout<<ctrl0.value<<endl;
}

//通过V4L2设置饱和度
void v4l2_set::v4l2_saturation(const char *file_name, int saturation)
{
    int Handle0=open(file_name,O_RDWR);
    struct v4l2_control saturation_s;
    saturation_s.id = V4L2_CID_SATURATION;
    saturation_s.value = saturation;
    ioctl(Handle0,VIDIOC_S_CTRL,&saturation_s);
}

//得到相机饱和度的值
void v4l2_set::Get_v4l2_saturation(const char *file_name)
{
    int Handle0=open(file_name,O_RDWR);
    struct v4l2_control saturation_s;
    saturation_s.id = V4L2_CID_SATURATION;
    std::cout<<"saturation: "<<saturation_s.value<<std::endl;
}


