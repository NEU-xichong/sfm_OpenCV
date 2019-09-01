//
// Created by xc on 19-6-4.
//

#ifndef FEATURE_COMMON_H
#define FEATURE_COMMON_H

#include <opencv2/opencv.hpp>
#include"const.h"


namespace FeatureGraph {

//判断所有的图片是否来自同一个规格
enum class Specification {
    Unified = 0,
    Disunity

};

enum class H_F {
    HomographyChoose = 0,
    FundanmentalChoose

};

enum class Camera_Parameters{
    //参数已知，用户提供
    Exsit_Para=0,

    //参数未知，从exif中提取
    No_Para
};

/*初始化的参数*/
struct Init_para{
    int min_num_inlier=100;
    double ransac_confidence=0.9999;
    double essential_error=4.0;
    double homography_error=12.0;
    double r_proj_max_error=2.0;
    double min_angle=4.0;
    Specification typeImg_Attr=Specification::Unified;

};
/*PnP求取的参数*/
struct PnP3D2D_para{
    int PnPMethod_=EPNP ;
    double ransac_confidence_=0.99;
    int max_error_=4;
    int iterative_opt_=10000;
    int inliers_num_=15;
};

struct point2d2d
{
    point2d2d(int feature_id1_,int feature_id2_, cv::Point2f p1_,cv::Point2f p2_):
             feature_id1(feature_id1_),feature_id2(feature_id2_),p1(p1_),p2(p2_){};
    int feature_id1;
    int feature_id2;
    cv::Point2f p1;
    cv::Point2f p2;
};


struct CandidatePair
{
    int view_1_id;
    int view_2_id;

    std::vector<point2d2d> matches;

    /*
    bool operator <(CandidatePair const& rhs) const{
        matches.size()<rhs.matches.size();
    }
    */
};

typedef std::vector<CandidatePair> CandidatePairs;


/*这个参数用户可以配置*/
struct Params
{
    double fx;
    double fy;
    double cx;
    double cy;

    double k1=0.0;
    double k2=0.0;
    double p1=0.0;
    double p2=0.0;

    struct Init_para    Init_para_;
    struct PnP3D2D_para PnP3D2D_para_;
};

/*初始化的返回值*/
struct Return_Info_Init
{
    bool is_succeed = false;                     
    size_t num_inliers = 0;                      
    double ave_residual = 0;                     
    cv::Mat R1;                                  
    cv::Mat t1;                                  
    cv::Mat R2;                                  
    cv::Mat t2;                                 
    std::vector<cv::Vec3d> points3D;             
    std::vector<double> tri_angles;              
    std::vector<double> residuals;               
    std::vector<bool> inlier_mask;               
};

/*pnp的返回值*/
struct Return_Info_P3d2d
{

    bool is_success=false;
    size_t num_point2D_3D_correspondences = 0;
    size_t num_inliers = 0;

    cv::Mat R;
    cv::Mat t;
    std::vector<double> residuals;
    std::vector<bool> inlier_mask;
};

/*三角量测的返回值*/
struct Return_Info_Tri
{
    bool is_succeed = false;
    cv::Vec3d point3D;
    double ave_residual = 0;
    std::vector<double> residuals;

    std::vector<bool> inliers_;
};

/*一些工具函数，单例模式*/
class Utils
{
public:

    static
    std::vector<cv::Vec2d> point2fToVec2ds(const std::vector<cv::Point2f> ptf)
    {};


    static
    cv::Vec2d point2fToVec2d(const cv::Point2f ptf)
    {   
        double x=static_cast<double>(ptf.x);
        double y=static_cast<double>(ptf.y);
        return cv::Vec2d(x,y);
    };


    static
    cv::Vec3d point3fToVec3d(const cv::Point3f ptf)
    {
        
        double x=static_cast<double>(ptf.x);
        double y=static_cast<double>(ptf.y);
        double z=static_cast<double>(ptf.z);
        return cv::Vec3d(x,y,z);
    };


    static
    std::vector<cv::Point3f> vec3fToPoint3f(const std::vector<cv::Vec3f> ptf)
    {
        std::vector<cv::Point3f> p3f;
        for(size_t i=0;i<ptf.size();++i)
        {
            float x=ptf[i][0];
            float y=ptf[i][1];
            float z=ptf[i][2];

            p3f.push_back(cv::Point3f(x,y,z));
        }
        
        return p3f;
    };
    static
    std::vector<cv::Point2f> vec2fToPoint2f(const std::vector<cv::Vec2f> ptf)
    {
        std::vector<cv::Point2f> p2f;
        for(size_t i=0;i<ptf.size();++i)
        {
            float x=ptf[i][0];
            float y=ptf[i][1];
            //float z=ptf[i][2];
            p2f.push_back(cv::Point2f(x,y));
        }
        
        return p2f;
    };

};


}


#endif //FEATURE_COMMON_H
