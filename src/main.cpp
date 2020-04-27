#include "basic_utils.h"
#include "map"

struct KeyFrame
{
    shared_ptr<cvMatT> img;
    vector<weak_ptr<cvMatT> > refFrame;
};

class DenseReconstructor
{
private:
    int current_id=0;
    int current_kf_id=0;
    std::map<int,shared_ptr<KeyFrame> > kf_mapping;
    shared_ptr<KeyFrame> getKFByID(int id);
    bool need_new_KF(int current_id,const vector<double>& wxyz_pos7)
    {
        auto pKF = getKFByID(current_kf_id);
        //if(co_vis(pKF,wxyz_pos7))
        //...
        if(current_id - current_kf_id > 10)
        {
            return true;
        }
        return false;
    }
public:

    DenseReconstructor()
    {

    }
    void iterate(shared_ptr<cvMatT> new_img_8UC1,const vector<double>& wxyz_pos7,bool& is_newKF);
    void updateKF_cpu(const cvMatT& new_img);

};

void DenseReconstructor::iterate(shared_ptr<cvMatT> new_img_8UC1,const vector<double>& wxyz_pos7,bool& is_newKF)
{
    //find refering KF;
    //for pts:track and update filter
    if(need_new_KF(current_id,wxyz_pos7))
    {
        shared_ptr<KeyFrame> kf_ptr(new(KeyFrame));
        kf_ptr->img = new_img_8UC1;
        this->kf_mapping[this->current_id] = kf_ptr;
        is_newKF = true;
        current_kf_id = current_id; // add new kf!
        current_id++;
        return;
    }
    is_newKF = false;
    updateKF_cpu(*new_img_8UC1);
    current_id++;
    return;
}
void DenseReconstructor::updateKF_cpu(const cvMatT &new_img)
{
    auto pKF = getKFByID(current_kf_id);
    //do epipolar search
}





int main()
{
    return 0;
}
