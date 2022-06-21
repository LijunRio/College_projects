#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <caffe_model.h>
#include <opencv2/core/utils/trace.hpp>
#include <opencv2/dnn.hpp>

int caffe_use(cv::Mat image)
{
    using namespace cv;
    using namespace std;
    using namespace cv::dnn;

    caffe_model caf_use;
    //cv::Mat image = cv::imread("1044.jpg");
    cv::cvtColor(image,image,CV_BGR2GRAY);
    cv::imshow("image", image);

    //初始化
    CV_TRACE_FUNCTION();
    //读取模型参数和模型结构文件
    String modelTxt = "lenet.prototxt";
    String modelBin = "roboArmor_iter_10000.caffemodel";

    //合成网络
    Net net = dnn::readNetFromCaffe(modelTxt, modelBin);
    //判断网络是否生成成功
    if (net.empty())
    {
        std::cerr << "Can't load network by using the following files: " << std::endl;
        exit(-1);
    }
    cerr << "net read successfully" << endl;

    if (image.empty())
    {
        std::cerr << "Can't read image" << std::endl;
        exit(-1);
    }
    cerr << "image read sucessfully" << endl;

/*	Mat inputBlob = blobFromImage(img, 1, Size(224, 224),
                                    Scalar(104, 117, 123)); */

    //构造blob，为传入网络做准备，图片不能直接进入网络
    cv::Mat inputBlob = blobFromImage(image, 1, Size(28, 28));

    cv::Mat prob;
    cv::TickMeter t;
    for (int i = 0; i < 10; i++)
    {
        CV_TRACE_REGION("forward");
        //将构建的blob传入网络data层
        net.setInput(inputBlob,"data");
        //计时
        t.start();
        //前向预测
        prob = net.forward("prob");
        //停止计时
        t.stop();
    }

    int classId;
    double classProb;
    //找出最高的概率ID存储在classId，对应的标签在classProb中
    caf_use.getMaxClass(prob, &classId, &classProb);

    //打印出结果
    std::vector<String> classNames = caf_use.readClassNames();
    std::cout << "Best class: #" << classId << " '" << classNames.at(classId) << "'" << std::endl;
    std::cout << "Probability: " << classProb * 100 << "%" << std::endl;
    //打印出花费时间
    std::cout << "Time: " << (double)t.getTimeMilli() / t.getCounter() << " ms (average from " << t.getCounter() << " iterations)" << std::endl;

    //便于观察结果
//    cv::waitKey(0);
    return classId;
}
