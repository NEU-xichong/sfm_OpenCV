//
// Created by xc on 19-6-4.
//
#include "../include/Camera.h"


namespace FeatureGraph
{

void Camera::getImageK() {


    K=(cv::Mat_<float>(3,3)<<focal_length,0,           prinpcipal_point.x,
                             0,           focal_length,prinpcipal_point.y,
                             0,           0,           1 );

}

void Camera::getR_t() {

    R_t=cv::Matx34f(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0),
                    R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1),
                    R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2));


}



}