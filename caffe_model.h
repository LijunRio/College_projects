#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class caffe_model
{
public:
    void getMaxClass(const cv::Mat &probBlob, int *classId, double *classProb);
    std::vector<String> readClassNames(const char *filename = "lab.txt");
};
