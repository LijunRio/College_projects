#include <caffe_model.h>
#include <fstream>
void caffe_model::getMaxClass(const cv::Mat &probBlob, int *classId, double *classProb)
{
    Mat probMat = probBlob.reshape(1, 1);
    Point classNumber;

    minMaxLoc(probMat, NULL, classProb, NULL, &classNumber);
    *classId = classNumber.x;
}

std::vector<String> caffe_model::readClassNames(const char *filename)
{
    std::vector<String> classNames;

    std::ifstream fp(filename);
    if (!fp.is_open())
    {
        std::cerr << "File with classes labels not found: " << filename << std::endl;
        exit(-1);
    }

    std::string name;
    while (!fp.eof())
    {
        std::getline(fp, name);
        if (name.length())
            classNames.push_back(name.substr(name.find(' ') + 1));
    }
    fp.close();
    return classNames;
}
