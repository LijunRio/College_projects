#include<iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include"detector_methods.h"
#include"coordinate_process.h"
#include<time.h>
#include<QDebug>
#include<QTime>
#include <v4l2_set.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include"serialport.h"
#include <pthread.h>
#include <unistd.h>
#include "Structure.h"
#include <mutex>
#define red 1
#define blue 2
#define frame_width 1280
#define frame_height 720

using namespace std;
using namespace cv;

cv::videocapture camera;
cv::mat frame;
std::mutex mut;

//多线程读摄像头函数
void *read(void* args)
{
	//用V4L2设置曝光和饱和度
    v4l2_set v4l21;
    v4l21.vel2_camera("/dev/video0",15);
    v4l21.v4l2_saturation("/dev/video0",100);
	//通过videocapture类获取图像
    cv::videocapture cap(0);
    cap.set(cv_cap_prop_frame_width,640);
    cap.set(cv_cap_prop_frame_height,480);
    if(!cap.isopened())
    {
        std::cout<<"open camera!!!"<<std::endl;
        exit(0);
    }
    while(1)
    {
		//获取图像
        mut.lock();
        qtime time;
        time.start();
        cap >> frame;
        mut.unlock();
        qdebug()<<"read_frame: "<<time.elapsed()<<"ms";//输出计时
    }
}

int main()
{//主摄像头
//创建线程
    int ret1 = 0;
    pthread_t id1;
	//多线程使用读图函数
    ret1 = pthread_create(&id1,NULL,read,NULL);
    if(ret1)
    {
        std::cout<<"creat pthread error"<<std::endl;
    }

    int save_count =0;

	//初始化串口
    SerialPort port;
    port.Open();
    //1是red
    //2是blue
    int flag=2;
    bool dia_flag=true;
    Point goal(295,280);


//===================接收颜色要求
    char *color_type = new char[9];
    bool color_flag = true;
    while(color_flag)
    {
        std::cout<<"Waiting~~~~"<<std::endl;
        port.Recv(color_type);
        if(color_type[0]=='R')
        {

            color_flag=false;
            flag=1;
        }
        else if(color_type[0]=='B')
        {


            color_flag=false;
            flag=2;
        }

    }
//===============================


    Vec3f pre_send(0,0,0);
    Point top_left(0,0);
    int count=0;
    while(true)
    {
        count++;
        cout<<"count: "<<count<<endl;

        camera>>frame;
        bool ROI=true;
        if(pre_send[0]==0)
        {
            ROI=false;
        }
        QTime time;
        time.start();
        Mat frame0;

        if(ROI)
        {

            double armour_width = pre_send[2]*0.5*2;
            double ratio = 0.75;
            int armour_height = armour_width*ratio*0.5;
            Point Center_Point(pre_send[0],pre_send[1]);
            int top = Center_Point.y - armour_height;
            if (top < 0)
            {
                top = 1;
            }
            int buttom = Center_Point.y + armour_height;
            if (buttom > frame_height)
            {
                buttom = frame_height-1;
            }

            int left = Center_Point.x - armour_width;
            if (left < 0)
            {
                left = 1;
            }
            int right = Center_Point.x + armour_width;
            if (right > frame_width)
            {
                right = frame_width-1;
            }
            Point point(left,top);
            top_left=point;
            frame0 = frame(cv::Range(top, buttom), cv::Range(left, right));
        }
        else
        {
            frame0=frame.clone();
             Point point(0,0);
             top_left=point;
        }

        methods cam0;
        cam0.set_img(frame0);

        //开关调节颜色===========================================

        Mat mask0;
        if(flag==1)
        {

            mask0=cam0.red_detector();//detect red color
            cv::imshow("mask0",mask0);
        }
        else if(flag==2)
        {
            mask0=cam0.blue_detector();//detect blue color
            cv::imshow("mask0",mask0);
        }

        //======================================================

        vector<vector<Point> > contours0;
        findContours(mask0,contours0,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        Vec3f armour_center0(2000,2000,2000);

//==========通过装甲板识别类得到坐标还要进行坐标转化==========================
        if(contours0.size()!=0)
        {
            armour_center0=cam0.bu_bing_blue(contours0,save_count);
         // 找不到东西的情况
            if(armour_center0[0]>frame_width)
            {
                 cout<<"no contours"<<endl;
                 Vec3f pre(0,0,0);
                 pre_send=pre;

            }
         //找到了目标的情况
            else if(armour_center0[0]<frame_width&& armour_center0[2]>0)
            {
                Point final_send(armour_center0[0]+top_left.x,armour_center0[1]+top_left.y);
                cout<<"finsl_send: "<<final_send<<endl;
				//坐标转化
                Vec3f s_pre(armour_center0[0]+top_left.x,armour_center0[1]+top_left.y,armour_center0[2]);
                pre_send=s_pre;
                double diameter=armour_center0[2];
                if(final_send.x<0||final_send.x>frame_width||final_send.y<0||final_send.y>frame_height)
                {
                    Vec3f pre(0,0,0);
                    pre_send=pre;
                }
                else
                {
                    if(abs(final_send.x-goal.x)<(diameter/3)&&abs(final_send.y-goal.y)<diameter/3)
                    {
                         waitKey(0);
                        cout<<final_send<<"aaa"<<endl;
                        coordinate_process send0;
//                        send0.a_send(final_send,port);
                    }
                    else
                    {
                         waitKey(0);
                        cout<<final_send<<endl;
                    }
                }

           }
            else
            {
                Vec3f pre(0,0,0);
                pre_send=pre;
            }
        }
        else
        {
            Vec3f pre(0,0,0);
            pre_send=pre;
        }
//==================================================

        cam0.show_img("frame");
        cv::waitKey(1);
        qDebug()<<"time:"<<time.elapsed()<<"ms";//输出计时

    }
    pthread_join(id,NULL); //等待线程全部结束
    return 0;

}
