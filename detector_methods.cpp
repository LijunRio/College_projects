#include "detector_methods.h"
#include <string>
#include"cctag_detector.h"
#include "caffe_use.h"
//==================set_img==========================
void methods::set_img(cv::Mat img)
{
    imgOriginal=img.clone();
    colsNumbers=imgOriginal.cols;
    rowsNumbers=imgOriginal.rows;
}
//===================draw_cross===========================
void methods::draw_cross()
{
    Point new_original_Point(colsNumbers/2,rowsNumbers/2);
    circle(imgOriginal,new_original_Point,2,Scalar(0,0,255));

    Point middle_left(0,rowsNumbers/2);
    Point midlle_right(colsNumbers,rowsNumbers/2);
    Point middle_up(colsNumbers/2,0);
    Point middle_botton(colsNumbers/2,rowsNumbers);

    line(imgOriginal,middle_left, midlle_right,Scalar(0,0,255),1,8,0);
    line(imgOriginal,middle_up, middle_botton,Scalar(0,0,255),1,8,0);
}
//======================blue_detector===============================
//直接在HSV颜色模型下识别蓝色
cv::Mat methods::blue_detector()
{
    blur(imgOriginal, imgOriginal, Size(5, 5));
    Mat imgHsv;
    cvtColor(imgOriginal, imgHsv, COLOR_BGR2HSV);
    //cv::blur(imgHsv,imgHsv,cv::Size(3,3));
    Mat mask;
    inRange(imgHsv, Scalar(50 + 6, 40 +7+5, 100 -18+28), Scalar(120, 255, 255), mask);
//    inRange(imgHsv, Scalar(56, 40 - 8, 100 - 18), Scalar(120, 255, 255), mask);

    Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat element1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    cv::dilate(mask,mask,element2);
    cv::erode(mask,mask,element1);
//    morphologyEx(mask, mask, MORPH_OPEN, element2);
//    morphologyEx(mask, mask, MORPH_CLOSE, element2);
    return mask;
}

//=======================blue bgr================================
//运用BGR通道相减来提取蓝色
cv::Mat methods::blue_BGR()
{
    Mat rImg,bImg,gImg,a,b,c,d;
    rImg.create(imgOriginal.rows,rowsNumbers, imgOriginal.type());
    bImg.create(imgOriginal.rows,rowsNumbers, imgOriginal.type());
    gImg.create(imgOriginal.rows,rowsNumbers, imgOriginal.type());
    a.create(imgOriginal.rows,rowsNumbers,imgOriginal.type());
    b.create(imgOriginal.rows,rowsNumbers,imgOriginal.type());
    c.create(imgOriginal.rows,rowsNumbers,imgOriginal.type());


    vector<Mat>results;
    split(imgOriginal,results);
    rImg=results[2];
    bImg=results[1];
    gImg=results[0];


    a=bImg-rImg;
    cvtColor(imgOriginal,b , CV_BGR2GRAY);

    inRange(a, 50-10+5,255, a);//0
    inRange(b,130-5,255, b);//130

    Mat element1 = getStructuringElement(MORPH_RECT, Size(7+5,7+5));//7//12
    dilate(b,b, element1);
    Mat element2 = getStructuringElement(MORPH_RECT, Size(7+7,7+7));//14
    dilate(a,a, element2);

    c = a.mul(b);

    Mat element3 = getStructuringElement(MORPH_RECT, Size(6,6));//5
    erode(c,c, element3);
    return c;
}
//=======================red_detector================================
//运用HSV检测红色，其中运用到图像取反操作。
//因为在HSV模型中红色的区域是在头和尾，是分开的。不利于调节阈值。
//所以通过取反把红色变为蓝色来识别
cv::Mat methods::red_detector()
{
    blur(imgOriginal, imgOriginal, Size(5, 5));
    Mat img_inv=~imgOriginal;
    Mat imgHsv;
    cvtColor(img_inv, imgHsv, COLOR_BGR2HSV);

    Mat mask;
    inRange(imgHsv, Scalar(80 , 40 - 8+0, 100 - 18), Scalar(120, 255, 255), mask);
//    inRange(imgHsv, Scalar(80 , 40 - 8, 100 - 18), Scalar(120-20, 255, 255), mask);
    Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, element2);
    morphologyEx(mask, mask, MORPH_CLOSE, element2);
    Mat element1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(mask,mask,element1);

    return mask;
}
//=======================red_lab_detector================================
//使用LAB模型来检测红色。
cv::Mat methods::red_lab_detector()
{
    Mat lab;
    cvtColor(imgOriginal, lab, CV_BGR2Lab);
    vector<cv::Mat> channels;
    split(lab,channels);
    Mat a_b=(channels.at(1)-channels.at(2));

    inRange(a_b, Scalar(50,50,50), Scalar(255, 255, 255), a_b);

    Mat element2 = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(a_b, a_b, MORPH_OPEN, element2);
    morphologyEx(a_b, a_b, MORPH_CLOSE, element2);
    return a_b;
}
//=======================red_bgr================================
//使用BGR模型来检测红色。
cv::Mat methods::red_BGR()
{
    Mat rImg,bImg,gImg,a,b,c,d;
    rImg.create(Size(imgOriginal.rows,imgOriginal.cols), imgOriginal.type());
    bImg.create(Size(imgOriginal.cols,imgOriginal.rows), imgOriginal.type());
    gImg.create(Size(imgOriginal.cols,imgOriginal.rows), imgOriginal.type());
    a.create(Size(imgOriginal.cols,imgOriginal.rows),imgOriginal.type());
    b.create(Size(imgOriginal.cols,imgOriginal.rows),imgOriginal.type());
    c.create(Size(imgOriginal.cols,imgOriginal.rows),imgOriginal.type());


    vector<Mat>results;
    split(imgOriginal,results);
    rImg=results[2];
    bImg=results[1];
    gImg=results[0];



    a=rImg-bImg-gImg;
    cvtColor(imgOriginal,b , CV_BGR2GRAY);

    threshold(b,b, 100, 255, THRESH_BINARY);//240
    threshold(a, a, 80, 255, THRESH_BINARY);//80

    Mat element1 = getStructuringElement(MORPH_RECT, Size(7+8,7+8));//7
    dilate(b,b, element1);
    Mat element2 = getStructuringElement(MORPH_RECT, Size(14+1,14+1));//14
    dilate(a,a, element2);

    c = a.mul(b);

    Mat element3 = getStructuringElement(MORPH_RECT, Size(5,5));//5
    erode(c,c, element3);
    return c;
}
//==================show_img====================================================
void methods::show_img(string a)
{
    imshow(a,imgOriginal);
    waitKey(1);
}

//==================blue===============================================
//装甲板识别逻辑方法
cv::Vec3f methods::bu_bing_blue(vector<vector<Point> > contours,int save_count)
{

    vector<RotatedRect> rectPoint(contours.size());
    int index_i=0;
    vector<Vec3f> lantern_message;
    cctag_detector det;
    vector<double> angle_lantern;
    vector<int> lantern_number;
    Mat det_img;
    imgOriginal.copyTo(det_img);

    //先把四个点重新排序
     for (index_i = 0; index_i < (int)contours.size(); index_i++)
     {
        //在进行x位置的相对排列
        rectPoint[index_i] = minAreaRect(contours[index_i]);
        Point2f draw_points[4];
        Point2f fourPoint2f[4];
        rectPoint[index_i].points(fourPoint2f);
        rectPoint[index_i].points(draw_points);

        //先把四个点以y由小到大排列
        for(int a=0;a<4;a++)
        {
            for(int b=a+1;b<4;b++)
            {
                if(fourPoint2f[a].y>fourPoint2f[b].y)
                {
                    Point2f temp;
                    temp=fourPoint2f[a];
                    fourPoint2f[a]=fourPoint2f[b];
                    fourPoint2f[b]=temp;
                }
            }
        }
        //在进行x位置的相对排列
        if(fourPoint2f[0].x>fourPoint2f[1].x)
        {
            Point2f temp0;
            temp0=fourPoint2f[0];
            fourPoint2f[0]=fourPoint2f[1];
            fourPoint2f[1]=temp0;
        }
        if(fourPoint2f[2].x>fourPoint2f[3].x)
        {
            Point2f temp1;
            temp1=fourPoint2f[2];
            fourPoint2f[2]=fourPoint2f[3];
            fourPoint2f[3]=temp1;
        }
         //计算排序后正常人所认为的长宽后把高大于宽的存储在lantern中
        double width_x=abs(fourPoint2f[1].x-fourPoint2f[0].x);
        double width_y=abs(fourPoint2f[1].y-fourPoint2f[0].y);
        double height_x=abs(fourPoint2f[2].x-fourPoint2f[0].x);
        double height_y=abs(fourPoint2f[2].y-fourPoint2f[0].y);
        double width=sqrt(pow(width_x,2)+pow(width_y,2));
        double height=sqrt(pow(height_x,2)+pow(height_y,2));

        if((width/height)>2)
        {
            continue;
        }

        double angle=rectPoint[index_i].angle;
        if(abs(angle)>25&&abs(angle)<70)
        {
            continue;
        }
        double y=rectPoint[index_i].center.y;
        if(y<120)
        {
            lantern_number.push_back(index_i);
        }
        cout<<"angle: "<<angle<<endl;
        //-----lantern_message-----------------
        double area=contourArea(contours[index_i] );
        Vec3f message(index_i,height,area);
        lantern_message.push_back(message);
        angle_lantern.push_back(angle);

        for (int k = 0; k < 4; k++)
        {
           line(imgOriginal, draw_points[k], draw_points[(k + 1) % 4], Scalar(0, 0, 255), 2, 8);
        }

    }


     //--------------exclude mini lantern------------------------

    if(lantern_number.size()>5)
    {
        Vec3f center(2000,2000,2000);
        return center;
    }

    //判断平行和y的相对位置存储所有预判的装甲板（存在误识别）
    vector<Vec3f> armour;
    for(int i=0;i<(int)lantern_message.size();i++)
    {
        for(int j=i+1;j<(int)lantern_message.size();j++)
        {
            double original_index_i=lantern_message[i][0];
            double original_index_j=lantern_message[j][0];

            double area1=lantern_message[i][2];
            double area2=lantern_message[j][2];

            double height_1=lantern_message[i][1];
            double height_2=lantern_message[j][1];

            double area_rate=area1/area2;
            double lantern_cha=abs(rectPoint[original_index_i].center.y-rectPoint[original_index_j].center.y);
            double cha=20;  //20
            double rate1=2.8;
            double rate2=0.2;
            double angle_1=angle_lantern[i];
            double angle_2=angle_lantern[j];
            cout<<"-----------angle_1: "<<angle_1<<"----------angle2: "<<angle_2<<endl;
            if((abs(angle_1)>5&&abs(angle_1)<20)&&(abs(angle_2)>65&&abs(angle_2)<85))
            {
                continue;
            }
            if((abs(angle_2)>5&&abs(angle_2)<20)&&(abs(angle_1)>65&&abs(angle_1)<85))
            {
                continue;
            }

            if(height_1<20||height_2<20)
            {
                cha=10;
            }
            else
            {
                cha=20;
            }

	
            if(lantern_cha<cha&&(area_rate>rate2&&area_rate<rate1)) //英雄看步兵
            {

                double diameter=sqrt(pow(rectPoint[original_index_i].center.x-rectPoint[original_index_j].center.x,2)+
                                                      pow(rectPoint[original_index_i].center.y-rectPoint[original_index_j].center.y,2));
                if(diameter<35)
                {
                    continue;
                }
                cout<<"d: "<<diameter<<endl;

                double average_height=(height_1+height_2)*0.5;
                cout<<"h: "<<average_height<<endl;
                double  bili=average_height/diameter;
                if(bili>0.8&&bili<1.8)
                {
                    continue;
                }
                if((diameter/average_height>2.5&&diameter/average_height<6)||(diameter/average_height>0.3&&diameter/average_height<2.5))
                {
                    Point armour_center=(rectPoint[original_index_i].center+rectPoint[original_index_j].center)*0.5;
                    circle(imgOriginal,armour_center,diameter/2,Scalar(0,255,255),1);//紫色
                    Vec3f armour_message(armour_center.x,armour_center.y,diameter);
                    armour.push_back(armour_message);
                }
            }
        }
    }

    //所有装甲板的选择判断选择最终装甲板

    //无选择
    if(armour.size()==0)
    {
        Vec3f center(2000,2000,2000);
        return center;
    }

    //一个选择
    else if(armour.size()==1)
    {
        Point center0(armour[0][0],armour[0][1]);
        cv::Mat image_roi;
        image_roi = det.image_cut(det_img,center0,armour[0][2],rowsNumbers,colsNumbers);
        bool flag = det.cctag_detect01(image_roi,save_count);


        if(flag)
        {
            circle(imgOriginal,center0,5,Scalar(0,0,255),5);//红色
            return armour[0];
        }
        else
        {
            Vec3f center(2000,2000,2000);
            return center;
        }
    }



    else
    {//近的存一组
        vector<Vec3f> near_armour;
        for(int i=0;i<(int)armour.size();i++)
        {
            if(armour[i][1]>imgOriginal.rows/2)
            {
                near_armour.push_back(armour[i]);
            }
        }
        //如果有近的装甲板
        if(near_armour.size()>0)
        {

            for(int i=0;i<near_armour.size();i++)
            {
                Point center(near_armour[i][0],near_armour[i][1]);


                cv::Mat image_roi;
                image_roi = det.image_cut(det_img,center,near_armour[i][2],rowsNumbers,colsNumbers);
				//没有更改贴纸的时候用cctag检测贴纸的圆形来最后判断装甲板
				//更改了贴纸后用caffe训练的数字模型来检测装甲板
                //bool flag = det.cctag_detect01(image_roi,save_count);
				bool flag = caffe_use(image_roi);

                if(flag)
                {
                    circle(imgOriginal,center,5,Scalar(0,0,255),5);//红色
                    return near_armour[i];
                }
            }
//            //都不是的情况
            Vec3f center(2000,2000,2000);
            return center;
        }
        //如果没有比较靠近的装甲板
        else
        {
            for(int i=0;i<(int)armour.size();i++)
            {
                Point center(armour[i][0],armour[i][1]);


                cv::Mat image_roi;
                image_roi = det.image_cut(det_img,center,armour[i][2],rowsNumbers,colsNumbers);
               // bool flag = det.cctag_detect01(image_roi,save_count);
				bool flag = caffe_use(image_roi);
                if(flag)
                {
                    Point center(armour[i][0],armour[i][1]);
                    circle(imgOriginal,center,5,Scalar(0,0,255),5);//红色
                    return armour[i];
                }
            }
            //都不是的情况
            Vec3f center(2000,2000,2000);
            return center;
        }
    }

 }
