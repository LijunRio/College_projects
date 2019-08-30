#pragma once
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <string>
using namespace cv;
using namespace std;
class methods
{
    public:
        void set_img(cv::Mat img);
        void draw_cross();
        Mat blue_BGR();
        Mat red_BGR();
        Mat red_detector();
        Mat blue_detector();
        Mat red_lab_detector();
        Vec3f bu_bing_blue(vector<vector<Point> >  contours,int save_count);
        void show_img(string a);
    private:
        cv::Mat imgOriginal;
        int colsNumbers;
        int rowsNumbers;
};
