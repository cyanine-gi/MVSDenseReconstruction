#ifndef BASIC_UTILS_H
#define BASIC_UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <memory>
#include <deque>

//Typedefs:


//#define USE_CV_UMAT
#ifdef USE_CV_UMAT
typedef cv::UMat cvMatT;
#else
typedef cv::Mat cvMatT;
#endif


using std::shared_ptr;
using std::vector;
using std::weak_ptr;

class OfflineDataLoader
{
private:
    int index = 0;

    std::string generate_path(int* id=nullptr)
    {
        if(id!=nullptr)
        {
            *id = index;
        }
        std::stringstream ss;
        ss<<"data/sequence1/"<<index<<".jpg";
        std::string ret_val;
        ss>>ret_val;
        index++;
        return ret_val;
    }
public:
    static shared_ptr<cvMatT> toGRAY(shared_ptr<cvMatT> pimg)
    {
        auto ret_val = std::make_shared<cvMatT>();
        cv::cvtColor(*pimg,*ret_val,cv::COLOR_BGR2GRAY);
        return ret_val;
    }
    shared_ptr<cvMatT> get_img(int* id)
    {
        auto path = this->generate_path(id);
        auto mat_ptr = new cv::Mat(cv::imread(path));//TODO:img undistort.
        shared_ptr<cv::Mat> m(mat_ptr);

#ifdef USE_CV_UMAT
        shared_ptr<cv::UMat> um(new cv::UMat(m->getUMat(cv::ACCESS_RW)
                                      ));
        return um;
#else
        return m;
#endif
    }
};

#endif
