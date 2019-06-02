//
// Created by xc on 19-5-30.
//

#ifndef FEATUREMATCH_FEATUREEXTRACTION_H
#define FEATUREMATCH_FEATUREEXTRACTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

//#include "../include/Image.h"

namespace FeatureGraph
{


class FeatureExtraction{



public:
    FeatureExtraction(const int &max_image_size=3200,
                      const int &max_num_features=10240)
                      :max_image_size_(max_image_size),max_num_features_(max_num_features){}

    void extractFeature(const cv::Mat &img,std::vector<cv::KeyPoint> &key_points,cv::Mat &des );


    //void keyPoitsToPoints(cv::)


private:

    int max_image_size_;
    int max_num_features_;


};





}




#endif //FEATUREMATCH_FEATUREEXTRACTION_H
