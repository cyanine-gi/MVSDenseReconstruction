#ifndef BASIC_UTILS_H
#define BASIC_UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <memory>
#include <deque>
#include <iostream>
#include <fstream>

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
    vector<vector<double> > pose_list;

    std::string generate_path(int* id=nullptr)
    {
        if(id!=nullptr)
        {
            *id = index;
        }
        std::stringstream ss;
        char buf[1000];
        memset(buf,0,1000);
        sprintf(buf,"data/sequence1/images/scene_%3.3d.png",index);
        //ss<<"data/sequence1/"<<index<<".jpg";
        std::string ret_val(buf);
        //ss>>ret_val;
        index++;
        return ret_val;
    }
public:
    OfflineDataLoader()
    {
        std::ifstream input_stream;
        input_stream.open("data/sequence1/first_200_frames_traj_over_table_input_sequence.txt");
        //scene_000.png 1.086410 4.766730 -1.449960 0.789455 0.051299 -0.000779 0.611661
        std::string line_input;
        while(std::getline(input_stream,line_input))
        {
            auto ss = std::stringstream(line_input);
            std::string img_path;
            ss>>img_path;
            double w,x,y,z,px,py,pz;
            vector<double> measurement;
            ss>>px;
            ss>>py;
            ss>>pz;
            ss>>w;
            ss>>x;
            ss>>y;
            ss>>z;
            measurement.push_back(w);measurement.push_back(x);measurement.push_back(y);measurement.push_back(z);
            measurement.push_back(px);measurement.push_back(py);measurement.push_back(pz);
            this->pose_list.push_back(measurement);
        }
    }
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
        return toGRAY(um);
#else
        return toGRAY(m);
#endif
    }
    vector<double> get_pose()
    {
        return this->pose_list[this->index];
    }
};

#endif
