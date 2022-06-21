#pragma once
#include <opencv2/opencv.hpp>


class cctag_detector
{
public:
	cctag_detector();
	~cctag_detector();
    cv::Mat image_cut(cv::Mat frame,cv::Point center,int armour_width,int rows,int cols);
    bool cctag_detect(cv::Mat imgROI);
    bool cctag_detect01(cv::Mat imgROI,int save_count);
    int  image_save(cv::Mat image,int number);


private:
	cv::Mat src_img;
	cv::Point Center_Point;
	int raduis;
};

