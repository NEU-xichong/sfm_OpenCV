//
// Created by xc on 19-6-3.
//

#ifndef FEATURE_INIT_PAIR_H
#define FEATURE_INIT_PAIR_H

#include "../include/common.h"
#include "../include/Image.h"
#include "../include/Camera.h"
namespace FeatureGraph
{


class Initializer
{

public:
    Initializer(const int min_num_inlier=100,
                const double ransac_confidence=0.9999,
                const double essential_error=4.0,
                const double homography_error=12.0
               ):min_num_inlier_(min_num_inlier),
                 ransac_confidence_(ransac_confidence),
                 essential_error_(essential_error),
                 homography_error_(homography_error){}

    void RecoverPose(pairImg &pair,Camera &Cam);


    //void FindHomography(pairImg &pair);

    //void FindF

    //void FindEssential(pairImg &pair);

    void RecoverPoseFromH(pairImg &pair,Camera &Cam);

    void RecoverPoseFromE(pairImg &pair,Camera &Cam);



private:


    int min_num_inlier_;
    double ransac_confidence_;
    double essential_error_;
    double homography_error_;

    //TODO:三角量测的阈值

};



}

#endif //FEATURE_INIT_PAIR_H
