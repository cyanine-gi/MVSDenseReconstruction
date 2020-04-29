#include "algorithms.h"


int main()
{
    //test epipolar search;
    //test inv-depth filter;
    //visualize


    auto pimg1 = std::make_shared<cvMatT>();
    auto pimg2 = std::make_shared<cvMatT>();
    *pimg1 = cv::imread("./data/test_epipolar_track/scene_000.png");
    *pimg2 = cv::imread("./data/test_epipolar_track/scene_020.png");

    auto p1_gray = OfflineDataLoader::toGRAY(pimg1);
    auto p2_gray = OfflineDataLoader::toGRAY(pimg1);
    vector<double> d1{0.789455, 0.051299, -0.000779, 0.611661 ,1.086410,4.766730 ,-1.449960};
    vector<double> d2{ 0.799432 ,0.060270, 0.033440, 0.596789,1.098910, 4.768230, -1.466230};


    cvMatT invdepth = cvMatT(p1_gray->rows, p1_gray->cols, CV_64F, MVSDR::init_invdepth);
    cvMatT invdepth_cov = cvMatT(p1_gray->rows, p1_gray->cols, CV_64F, MVSDR::init_invdepth);
    //const cvMatT& img_ref,const cvMatT& img_curr,cvMatT& inv_depth_map,cvMatT&inv_depth_cov_map
    MVSDR::updateImgCPU(d1,d2,*p1_gray,*p2_gray,invdepth,invdepth_cov);

    cvMatT output;
    MVSDR::convertInvDepthToDepth(invdepth,output);
    cv::imwrite("output.png",output);
    std::cout <<"Exit normally!"<<std::endl;
    return 0;
}

