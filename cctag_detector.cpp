#include "cctag_detector.h"
#include<iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;

cctag_detector::cctag_detector()
{
}


cctag_detector::~cctag_detector()
{
}

cv::Mat cctag_detector::image_cut(cv::Mat frame,cv::Point Center_Point, int armour_width,int rows,int cols)
{
    armour_width = armour_width*0.5;
    double ratio = 2;//0.75;
    int armour_height = armour_width*ratio;
    src_img = frame.clone();
    int top = Center_Point.y - armour_height*0.6;
    if (top < 0)
    {
        top = 1;
    }
    int buttom = Center_Point.y + armour_height*0.8;
    if (buttom > rows)
    {
        buttom = rows-1;
    }

    int left = Center_Point.x - armour_width*0.7;
    if (left < 0)
    {
        left = 1;
    }
    int right = Center_Point.x + armour_width*0.7;
    if (right > cols)
    {
        right = cols-1;
    }

    cv::Mat imgROI = frame(cv::Range(top, buttom), cv::Range(left, right));
    return imgROI;
}


bool cctag_detector::cctag_detect(cv::Mat im)
{
    clock_t t1 = clock();
    cv::GaussianBlur(im, im, cv::Size(3, 3), 0);

    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 240;
    params.thresholdStep = 5;
    params.minRepeatability = 1;

    params.filterByColor = true;     //�ߵ���ɫ�����Ʊ���
    params.blobColor = 255;          //��ʾֻ��ȡ��ɫ�ߵ㣻�����ñ���Ϊ255����ʾֻ��ȡ��ɫ�ߵ�

                                     // Filter by Area.
    params.filterByArea = true;
    int image_size = im.size().height * im.size().width;
    params.minArea = (1 / 10.0)*image_size;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.8;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1;


    // Storage for blobs
    std::vector<cv::KeyPoint> keypoints;


#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
    cout << "using OpenCV 2" << endl;
    // Set up detector with params
    SimpleBlobDetector detector(params);

    // Detect blobs
    detector.detect(im, keypoints);
#else

    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs
    detector->detect(im, keypoints);
#endif

    std::cout << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
    // the size of the circle corresponds to the size of blob

    cv::Mat im_with_keypoints;
    drawKeypoints(im, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // Show blobs
    imshow("keypoints", im_with_keypoints);
    if (keypoints.size() > 0)
    {
        std::cout<<"True!!!"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"False!!!"<<std::endl;
        return false;
    }
}



bool cctag_detector::cctag_detect01(cv::Mat imgROI,int save_count)
{
    cv::Mat mask;
    cv::Mat mask_copy;
    Mat img_gray,mat_mean,mat_stddev;
    cvtColor(imgROI,img_gray,CV_BGR2GRAY);
    meanStdDev(img_gray,mat_mean,mat_stddev);\
    double m,s;
    m=mat_mean.at<double>(0,0);
    s=mat_stddev.at<double>(0,0);
    cout<<"grave_mean_balue: "<<m<<endl;
    cout<<"cov_value: "<<s<<endl;
    if(m>50||s>25)
    {
       return false;
    }
    else
    {
        return true;
    }


    cv::imshow("mask_copy",mask_copy);
    cv::imshow("mask", mask);
    cv::imshow("imgROI", imgROI);
}

int   cctag_detector::image_save(cv::Mat image,int number)
{
    std::string filename;
    std::string path;
    std::stringstream file;
    file<<number;
    file>>filename;
    filename += ".jpg";
    if(image.empty())
    {
        return 0;
    }
    path = "mistake/"+filename;
    std::cout<<"filename: "<<path<<std::endl;
    cv::imwrite(path,image);
    return 0;
}

