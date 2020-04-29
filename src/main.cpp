#include "basic_utils.h"
#include "algorithms.h"
#include "map"

struct KeyFrame
{
    shared_ptr<cvMatT> img;
    vector<double> wxyz_pos_;
    shared_ptr<cvMatT> pInvDepth = nullptr;
    shared_ptr<cvMatT> pInvDepthCov = nullptr;
    shared_ptr<cvMatT> pValidMap = nullptr;
    void generate_valid_map()
    {
        cv::Mat blured,laplacian_map;
        this->pValidMap = std::make_shared<cvMatT>(img->rows,img->cols,CV_8U,0);
        cv::GaussianBlur(*img,blured,cv::Size(5,5),3);
        cv::Laplacian(blured,laplacian_map,CV_32F,3);
        for(int y=0;y<laplacian_map.rows;y++)
        {
            for(int x=0;x<laplacian_map.cols;x++)
            {
                if(laplacian_map.at<float>(y,x)>5)
                    //if(laplacian_map.at<float>(y,x)>10) //best thres val between 5 and 10.
                {
                    this->pValidMap->at<uint8_t>(y,x) = 255;
                }
            }
        }
        cv::imwrite("laplacian_valid_map.png",*pValidMap);
    }
    //vector<weak_ptr<cvMatT> > refFrame;
};

class DenseReconstructor
{
private:
    int current_id=0;
    int current_kf_id=0;
    std::map<int,shared_ptr<KeyFrame> > kf_mapping;
    const int KF_DIST = 20;
    shared_ptr<KeyFrame> getKFByID(int id)
    {
        return this->kf_mapping.at(id);
    }
    bool need_new_KF(int current_id,const vector<double>& wxyz_pos7)
    {
        if(current_id == 0)
        {
            return true;
        }
        auto pKF = getKFByID(current_kf_id);
        //if(co_vis(pKF,wxyz_pos7))
        //...
        if(current_id - current_kf_id > KF_DIST)
        {
            return true;
        }
        return false;
    }
    void saveKF(int kfid)
    {
        auto pOldKF = getKFByID(kfid);
        std::stringstream filename_ss;
        filename_ss<<"output_kf_"<<kfid<<".png";
        std::string path = filename_ss.str();
        cvMatT output;
        MVSDR::convertInvDepthToDepth(*pOldKF->pInvDepth,output);
        cv::imwrite(path,output);
    }
public:

    DenseReconstructor()
    {

    }
    void iterate(shared_ptr<cvMatT> new_img_8UC1,const vector<double>& wxyz_pos7,bool& is_newKF);
    void updateKF_cpu(const cvMatT& new_img,vector<double> wxyz_pos_curr);

};


void DenseReconstructor::iterate(shared_ptr<cvMatT> new_img_8UC1,const vector<double>& wxyz_pos7,bool& is_newKF)
{
    //find refering KF;
    //for pts:track and update filter
    if(need_new_KF(current_id,wxyz_pos7))
    {
        shared_ptr<KeyFrame> kf_ptr(new(KeyFrame));
        kf_ptr->img = new_img_8UC1;
        kf_ptr->wxyz_pos_ = wxyz_pos7;
        kf_ptr->generate_valid_map();


        this->kf_mapping[this->current_id] = kf_ptr;

        is_newKF = true;
        if(current_id!=0)
        {
            saveKF(current_kf_id);//save old kf output.
        }
        current_kf_id = current_id; // add new kf!
        current_id++;
        return;
    }
    is_newKF = false;
    updateKF_cpu(*new_img_8UC1,wxyz_pos7);
    current_id++;
    return;
}
void DenseReconstructor::updateKF_cpu(const cvMatT &new_img,vector<double> wxyz_pos_curr)
//new img is gray.
{
    auto pKF = getKFByID(current_kf_id);
    auto pRefImg = pKF->img;


    const vector<double>& d_ref = pKF->wxyz_pos_;
    const vector<double>& d_curr = wxyz_pos_curr;

    if(pKF->pInvDepth == nullptr)
    {
        pKF->pInvDepth =shared_ptr<cvMatT>(new cvMatT(pRefImg->rows, pRefImg->cols, CV_64F, MVSDR::init_invdepth));
        pKF->pInvDepthCov = shared_ptr<cvMatT>(new cvMatT(pRefImg->rows, pRefImg->cols, CV_64F, MVSDR::init_invdepth_cov));
    }
    //const cvMatT& img_ref,const cvMatT& img_curr,cvMatT& inv_depth_map,cvMatT&inv_depth_cov_map
    MVSDR::updateImgCPU(d_ref,d_curr,*pRefImg,new_img,*pKF->pInvDepth,*pKF->pInvDepthCov,pKF->pValidMap);
    return;
}





int main()
{
    OfflineDataLoader odl;
    DenseReconstructor DR;
    for(int i = 0;i< 100;i++)
    {
        int index_;
        auto img = odl.get_img(&index_);
        auto pose = odl.get_pose();

        bool current_frame_is_kf;
        DR.iterate(img,pose,current_frame_is_kf);
        if(current_frame_is_kf)
        {
            std::cout<<"Current Frame:"<<index_<<" is a Keyframe!"<<std::endl;
        }
        else
        {
            std::cout<<"Current Frame:"<<index_<<" is a ordinary frame!"<<std::endl;
        }

    }

    return 0;
}
