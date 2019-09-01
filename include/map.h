#ifndef FEATURE_MAP_H
#define FEATURE_MAP_H


#include<map>
#include<unordered_map>
#include<vector>
#include"tracks.h"


namespace FeatureGraph{


typedef int point3d_idx;

struct registered_images_
{
    registered_images_(const cv::Mat &R,const cv::Mat &t):R_(R),t_(t){};
    registered_images_(){};
    cv::Mat R_;
    cv::Mat t_;
};

struct point3d_idx_error{
    int      idx;
    double   err;
};

struct trackElement
{
    trackElement(int image_id_,int point2d_idx_):image_id(image_id_),point2d_idx(point2d_idx_){};
    int image_id;
    int point2d_idx;
};

typedef std::vector<trackElement> track_;

struct point3d_track{

    cv::Vec3d   pos_;
    cv::Vec3b   color_;
    track_      tracks;

    double      err;

    //重建的三维点对应的索引
    point3d_idx idx_;


};

class Map{

public:
    Map();

    void AddPoint(const cv::Vec3d& point_,const track_ m_track_,const cv::Vec3b color,const double err);
    void AddExistPoint(const point3d_idx& point3D_idx_, const trackElement& track_el, const double& error);


    //添加图像的位姿
    void AddImagePose(const int image_id,const cv::Mat R,const cv::Mat t);

    point3d_idx get_point3d_idx();

    std::vector<int> get_regist_img();

    //std::unordered_map<int,registered_images_> get_regist_img_pose();
    registered_images_ get_regist_img_pose(const int img_id);


    


private:

    //已经重建的三维点的索引
    point3d_idx num_point3d_idx;

    //地图中已经重建的三维点，map里的int对应num_point3d_idx
    std::unordered_map<point3d_idx,point3d_track> point3ds;


    //已经求取的图像的位姿
    std::unordered_map<int,registered_images_>images_;

    //已经注册的图像
    std::vector<int> registered_;


};

}

#endif