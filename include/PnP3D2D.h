//
// Created by xc on 19-6-8.
//

#ifndef FEATURE_PNP3D2D_H
#define FEATURE_PNP3D2D_H

#include<opencv2/opencv.hpp>
#include"common.h"
#include"const.h"
#include"init.h"

namespace FeatureGraph
{


class PnP3D2D
{
public:

    PnP3D2D( const int PnPMethod_=EPNP ,
             const double ransac_confidence_=0.99,
             const int max_error_=4,
             const int iterative_opt_=10000,
             const int inliers_num_=15
           ):PnPMethod(PnPMethod_),
            ransac_confidence(ransac_confidence_),
             max_error(max_error_),
             iterative_opt(iterative_opt_),
             inliers_num(inliers_num_)
             {};


    Return_Info_P3d2d PnP3d2dSolve(std::vector<cv::Point3f> point3ds,std::vector<cv::Point2f> point2ds,cv::Mat K_);

private:

    //pnp求解的内参
    int PnPMethod;
    double ransac_confidence;
    int max_error;
    int iterative_opt;

    int inliers_num;

    //相机内参
    //cv::Mat Cam_K;

};


}
#endif //FEATURE_PNP3D2D_H
