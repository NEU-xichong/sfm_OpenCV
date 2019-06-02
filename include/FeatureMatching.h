//
// Created by xc on 19-5-30.
//

#ifndef FEATUREMATCH_FEATUREMATCHING_H
#define FEATUREMATCH_FEATUREMATCHING_H

#include <opencv2/opencv.hpp>

namespace FeatureGraph
{

class FeatureMatching
{
public:
    FeatureMatching(const double distanceRatio=0.8,const double max_distance=200)
                   :distanceRatio_(distanceRatio),max_distance_(max_distance){}



    void matchFeatures(cv::Mat & desc1,cv::Mat & desc2,std::vector<cv::DMatch> & good_matches);

    bool geometric_verif_F(const std::vector<cv::Point2f>& pts1,
            const std::vector<cv::Point2f>& pts2,
            const std::vector<cv::DMatch> matches,
            std::vector<cv::DMatch> &good_matches,
            cv::Mat & Fundamental);

private:

    double distanceRatio_;
    double max_distance_;

};

}

#endif //FEATUREMATCH_FEATUREMATCHING_H
