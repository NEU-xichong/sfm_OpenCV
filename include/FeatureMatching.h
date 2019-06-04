//
// Created by xc on 19-5-30.
//

#ifndef FEATUREMATCH_FEATUREMATCHING_H
#define FEATUREMATCH_FEATUREMATCHING_H

#include <opencv2/opencv.hpp>
#include "../include/common.h"

namespace FeatureGraph
{

class FeatureMatching
{
public:
    FeatureMatching(const double distanceRatio=0.8,const double max_distance=200)
                   :distanceRatio_(distanceRatio),max_distance_(max_distance){}



    /**
     * @brief 进行了交叉匹配验证
     * @param desc1
     * @param desc2
     * @param good_matches
     */
    void matchFeatures(cv::Mat & desc1,cv::Mat & desc2,std::vector<cv::DMatch> & good_matches);

    /**
     *
     * @param pts1
     * @param pts2
     * @param matches
     * @param good_matches 满足内点的匹配关系
     * @param H_F_CHOOSE   枚举类型，判断该匹配对是满足单应矩阵还是基础矩阵（计算两种矩阵，判断内点的比例）
     * @param Fundamental  基础矩阵
     * @param Homography   单应矩阵
     * @return
     */
    bool geometric_verif_F_H(const std::vector<cv::Point2f>& pts1,
            const std::vector<cv::Point2f>& pts2,
            const std::vector<cv::DMatch> matches,
            std::vector<cv::DMatch> &good_matches,
            H_F &H_F_CHOOSE,
            cv::Mat &Fundamental,
            cv::Mat &Homography);

private:

    double distanceRatio_;
    double max_distance_;

};

}

#endif //FEATUREMATCH_FEATUREMATCHING_H
