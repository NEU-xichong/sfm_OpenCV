//
// Created by xc on 19-6-4.
//

#ifndef FEATURE_CAMERA_H
#define FEATURE_CAMERA_H


#include <opencv2/opencv.hpp>
#include "../include/Image.h"


namespace FeatureGraph
{

class Camera
{

public:

    Camera(const int imgid,const cv::Matx34f Rt=cv::Matx34f::eye()):imgId_(imgid),R_t(Rt){}


    //void computeE(const Parameter &parameter=Parameter::Unified,);

    void get_Focal_Length();

    void getPP();

    void getImageK();

    void getR_t();


public:


    //图像id
    int imgId_;

    //相机内参
    cv::Mat K;

    //相机的焦距
    int focal_length;


    cv::Point2d prinpcipal_point;

    //相机内参的逆
    cv::Mat Kinv;

    //相机的全局旋转
    cv::Mat R;

    //相机的全局平移
    cv::Mat t;

    cv::Matx34f R_t;

};



}





#endif //FEATURE_CAMERA_H
