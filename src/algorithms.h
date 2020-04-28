#ifndef MVS_DR_ALGORITHMS_H
#define MVS_DR_ALGORITHMS_H

#include "basic_utils.h"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace MVSDR{

using Sophus::SE3d;
using namespace Eigen;


//constant values:

const double fx = 376.0;       // 相机内参
const double fy = 376.0*-1;
const double cx = 376.0;
const double cy = 240;//cy = height(480) - cy_original(240)


//const int ncc_window_size = 3;    // NCC 取的窗口半宽度
const int ncc_window_size = 2;    // NCC 取的窗口半宽度
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1); // NCC窗口面积
const double min_cov = 0.1;     // 收敛判定：最小方差
const double max_cov = 3;      // 发散判定：最大方差
const int max_half_len = 50;

const int border_size = 10;

const double init_invdepth = 1;
const double init_invdepth_cov = 0.5;

// 像素到相机坐标系
inline double NCC(
    const cvMatT &ref, const cvMatT &curr,
    const Vector2d &pt_ref, const Vector2d &pt_curr);
inline Vector3d px2cam(const Vector2d& px) {
    return Vector3d(
        (px(0, 0) - cx) / fx,
        (px(1, 0) - cy) / fy,
        1
    );
}

// 相机坐标系到像素
inline Vector2d cam2px(const Vector3d p_cam) {
    return Vector2d(
        p_cam(0, 0) * fx / p_cam(2, 0) + cx,
        p_cam(1, 0) * fy / p_cam(2, 0) + cy
    );
}
inline bool insideImg(const Vector2d &pt,const cvMatT& mat) {
    return (pt(0, 0) >= border_size && pt(1, 0) >= border_size
           && pt(0, 0) < mat.cols -border_size && pt(1, 0) < mat.rows-border_size);
}
inline double getBilinearInterpolatedValue(const cvMatT &img, const Vector2d &pt) {
    uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
    double xx = pt(0, 0) - floor(pt(0, 0));
    double yy = pt(1, 0) - floor(pt(1, 0));
    return ((1 - xx) * (1 - yy) * double(d[0]) +
            xx * (1 - yy) * double(d[1]) +
            (1 - xx) * yy * double(d[img.step]) +
            xx * yy * double(d[img.step + 1])) / 255.0;
}
inline bool epipolarSearch(const cvMatT& img_ref,const cvMatT& img_curr,
                           const cvMatT& invdepth_mu,const cvMatT& invdepth_cov,
                           const SE3d& T_rel,int x,int y,
                           Vector2d &pt_curr, Vector2d &epipolar_direction);
inline bool updateInvDepth(
        const Vector2d &pt_ref,
        const Vector2d &pt_curr,
        const SE3d &T_rel_inv,
        const Vector2d &epipolar_direction,
        cvMatT &invdepth,
        cvMatT &invdepth_cov2);


void updateImg(const vector<double>& wxyz_pos_ref,const vector<double>& wxyz_pos_curr,const cvMatT& img_ref,const cvMatT& img_curr,cvMatT& inv_depth_map,cvMatT&inv_depth_cov_map)
{
    SE3d T_ref=SE3d(Quaterniond(wxyz_pos_ref[0], wxyz_pos_ref[1], wxyz_pos_ref[2], wxyz_pos_ref[3]),
                     Vector3d(wxyz_pos_ref[4],wxyz_pos_ref[5],wxyz_pos_ref[6]));
    SE3d T_curr=SE3d(Quaterniond(wxyz_pos_curr[0], wxyz_pos_curr[1], wxyz_pos_curr[2], wxyz_pos_curr[3]),
                     Vector3d(wxyz_pos_curr[4],wxyz_pos_curr[5],wxyz_pos_curr[6]));
    SE3d T_rel = T_curr*T_ref.inverse();
    SE3d T_rel_inv = T_rel.inverse();
//    shared_ptr<cvMatT> pDepth(new cvMatT(img_ref.rows, img_ref.cols, CV_64F, init_invdepth));             // 深度图
//    shared_ptr<cvMatT> pDepth_cov2(new cvMatT(img_ref.rows, img_ref.cols, CV_64F, init_invdepth_cov));         // 深度图方差

    //Matrix3d EssentialMat = t^*R
    Vector2d pt_curr,epipolar_direction;


    for(int y = border_size;y<img_ref.rows-border_size;y++)
    {
        for(int x = border_size;x<img_ref.cols-border_size;x++)
        {

            bool success = epipolarSearch(img_ref,img_curr,inv_depth_map,inv_depth_cov_map,T_rel,x,y,pt_curr,epipolar_direction);
            if(!success)
            {
                continue;
            }
            updateInvDepth(Vector2d(x,y),pt_curr,T_rel_inv,epipolar_direction,inv_depth_map,inv_depth_cov_map);
        }
    }
}
inline bool epipolarSearch(const cvMatT& img_ref,const cvMatT& img_curr,
                           const cvMatT& invdepth_mu,const cvMatT& invdepth_cov,
                           const SE3d& T_rel,int x,int y,
                           Vector2d &pt_curr, Vector2d &epipolar_direction)
{
    const Vector2d& uv_ref = Vector2d(x,y);
    const double invdepth_mu_ = invdepth_mu.at<double>(y,x);
    const double invdepth_cov_ = invdepth_cov.at<double>(y,x);
    Vector3d f_ref = px2cam(uv_ref);
    f_ref.normalize();
    Vector3d P_ref = f_ref / invdepth_mu_;    // 参考帧的 P 向量
    Vector2d px_mean_curr = cam2px(T_rel * P_ref); // 按深度均值投影的像素
    double invd_min = invdepth_mu_ - 3 * invdepth_cov_, invd_max =invdepth_mu_ + 3 * invdepth_cov_;

    if (invd_max > 10)
    {
        invd_max = 10;//0.1m ~ 10m
    }
    if(invd_min<0.1)
    {
        invd_min = 0.1;
    }

    Vector2d px_min_curr = cam2px(T_rel * (f_ref / invd_max));    // 按最小深度投影的像素
    Vector2d px_max_curr = cam2px(T_rel * (f_ref / invd_min));    // 按最大深度投影的像素

    Vector2d epipolar_line = px_max_curr - px_min_curr;    // 极线（线段形式）
    epipolar_direction = epipolar_line;        // 极线方向
    epipolar_direction.normalize();
    double half_length = 0.5 * epipolar_line.norm();    // 极线线段的半长度
    if (half_length > max_half_len)
    {
        half_length = max_half_len;   // 我们不希望搜索太多东西
    }
    // 在极线上搜索，以深度均值点为中心，左右各取半长度
    double best_ncc = -1.0;
    Vector2d best_px_curr;
    for (double l = -half_length; l <= half_length; l += 0.7) { // l+=sqrt(2)
        Vector2d px_curr = px_mean_curr + l * epipolar_direction;  // 待匹配点
        if (!insideImg(px_curr,img_ref))
        {
            continue;
        }
        // 计算待匹配点与参考帧的 NCC
        double ncc = NCC(img_ref, img_curr, uv_ref, px_curr);
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_px_curr = px_curr;
        }
    }
    if (best_ncc < 0.85f)      // 只相信 NCC 很高的匹配
    {
        return false;
    }
    pt_curr = best_px_curr;
    return true;
}

inline bool updateInvDepth(
        const Vector2d &pt_ref,
        const Vector2d &pt_curr,
        const SE3d &T_rel_inv,
        const Vector2d &epipolar_direction,
        cvMatT &invdepth,
        cvMatT &invdepth_cov2)
{
    SE3d T_R_C=T_rel_inv;
    Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    Vector3d f_curr = px2cam(pt_curr);
    f_curr.normalize();

    // 方程
    // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
    // f_ref_current_axes = R_RC * f_cur
    // 转化成下面这个矩阵方程组
    // => [ f_ref^T f_ref, -f_ref^T f_ref_current_axes ] [d_ref]   [f_ref^T t]
    //    [ f_ref_current_axes^T f_ref, -f_ref_current_axes^T f_ref_current_axes      ] [d_cur] = [f_ref_current_axes^T t   ]
    Vector3d t = T_R_C.translation();
    Vector3d f_ref_current_axes = T_R_C.so3() * f_curr;
    Vector2d b = Vector2d(t.dot(f_ref), t.dot(f_ref_current_axes));
    Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f_ref_current_axes);//(cos theta; theta is the angle relative to 3dpt reprojected in epipolar triangle.)
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f_ref_current_axes.dot(f_ref_current_axes);
    Vector2d ans = A.inverse() * b;
    Vector3d xm = ans[0] * f_ref;           // ref 侧的结果
    Vector3d xn = t + ans[1] * f_ref_current_axes;          // cur 结果
    Vector3d p_esti = (xm + xn) / 2.0;      // P的3d位置，取两者的平均
    double depth_estimation = p_esti.norm();   // 深度值
    double current_invdepth = 1.0/depth_estimation;

    // 计算inv_depth不确定性（以一个像素为误差）
    //How much effort will one pixel err do to the depth estimation( or, inv-depth estimation) ?
    Vector3d p = f_ref * depth_estimation;
    Vector3d a = p - t;
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos(f_ref.dot(t) / t_norm);
    double beta = acos(-a.dot(t) / (a_norm * t_norm));
    Vector3d f_curr_prime = px2cam(pt_curr + epipolar_direction);
    f_curr_prime.normalize();
    double beta_prime = acos(f_curr_prime.dot(-t) / t_norm);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    double d_cov = p_prime - depth_estimation;
    double invdepth_cov = -d_cov/(depth_estimation*depth_estimation);
    double invd_cov2 = invdepth_cov*invdepth_cov;

    double mu_original = invdepth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double sigma_squared_original = invdepth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];

    double mu_fuse = (invd_cov2 * mu_original + sigma_squared_original * current_invdepth) / (sigma_squared_original + invd_cov2);
    double sigma_fuse2 = (sigma_squared_original * invd_cov2) / (sigma_squared_original + invd_cov2);

    invdepth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
    invdepth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;

    return true;

}





double NCC(
        const cvMatT &ref, const cvMatT &curr,
        const Vector2d &pt_ref, const Vector2d &pt_curr) {
    // 零均值-归一化互相关
    // 先算均值
    double mean_ref = 0, mean_curr = 0;
    vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
    for (int x = -ncc_window_size; x <= ncc_window_size; x++)
        for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
            double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
            mean_ref += value_ref;

            double value_curr = getBilinearInterpolatedValue(curr, pt_curr + Vector2d(x, y));
            mean_curr += value_curr;

            values_ref.push_back(value_ref);
            values_curr.push_back(value_curr);
        }

    mean_ref /= ncc_area;
    mean_curr /= ncc_area;

    // 计算 Zero mean NCC
    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < values_ref.size(); i++) {
        double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-5);   // 防止分母出现零
}

void convertInvDepthToDepth(const cvMatT& invD,cvMatT& d)
{
    d = cvMatT(invD.rows,invD.cols,CV_8U);
    for(int y = 0;y<d.rows;y++)
    {
        for(int x = 0;x<d.cols;x++)
        {
            if(y%10 == 0&&x%10 == 0)
            {

                std::cout<<"val:"<<invD.at<double>(y,x)<<std::endl;
            }
            d.at<uint8_t>(y,x) = (1.0/invD.at<double>(y,x))*100;
        }
    }

}





}

#endif
