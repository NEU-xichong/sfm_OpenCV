//
// Created by xc on 19-6-3.
//

#ifndef FEATURE_INIT_PAIR_H
#define FEATURE_INIT_PAIR_H

#include "common.h"
#include "Image.h"
#include "Camera.h"
namespace FeatureGraph
{


class Initializer
{

public:
    Initializer(const int min_num_inlier=100,
                const double ransac_confidence=0.9999,
                const double essential_error=4.0,
                const double homography_error=12.0,
                const double r_proj_max_error=2.0,
                const double min_angle=4.0,
                const Specification typeImg_Attr=Specification::Unified
               ):min_num_inlier_(min_num_inlier),
                 ransac_confidence_(ransac_confidence),
                 essential_error_(essential_error),
                 homography_error_(homography_error),
                 r_proj_max_error_(r_proj_max_error),
                 min_angle_(min_angle),
                 type_(typeImg_Attr){};

    //bool RecoverPoseInitPair(pairImg &pair,std::vector<Camera> &Cam);


    //void FindHomography(pairImg &pair);

    //void FindF

    //void FindEssential(pairImg &pair);

    void RecoverPoseFromH(pairImg &pair,Camera &Cam);

    Return_Info_Init RecoverPoseFromE(const std::vector<point2d2d>& ,const cv::Mat&);

    //bool RecoverPoseFromE(cv::Vec2f p2d1,cv::Vec2f p2d2,);


    cv::Vec3d Triangulate(const cv::Mat P1, const cv::Mat P2, const cv::Vec2d point1, const cv::Vec2d point2);

    //函数重载
    Return_Info_Tri Triangulate(const std::vector<cv::Mat>& Rs,
                                const std::vector<cv::Mat>& ts,
                                const std::vector<cv::Vec2d>& points2D,
                                const cv::Mat &K_);
    

    cv::Vec3d TriangulateMultiviewPoint(const std::vector<cv::Mat>& Rs,
                                        const std::vector<cv::Mat>& ts,
                                        const std::vector<cv::Vec2d>& points2D,
                                        const cv::Mat &K_);

    static
    bool PositiveDepth(const cv::Vec3d point3d,
                       const cv::Mat R1,
                       const cv::Mat t1,
                       const cv::Mat R2,
                       const cv::Mat t2);

    static
    double ReprojectPairError(const cv::Vec3d point3d,
                              const cv::Vec2d point2d1,
                              const cv::Vec2d point2d2,
                              const cv::Mat R1,
                              const cv::Mat t1,
                              const cv::Mat R2,
                              const cv::Mat t2,
                              const cv::Mat K);
    static
    double CalculateReprojectError(const cv::Vec3d point3d,
                                   const cv::Vec2d point2d,
                                   const cv::Mat R,
                                   const cv::Mat t,
                                   const cv::Mat K);

    static
    double CalculateAngle(const cv::Vec3d point3d,
                          const cv::Mat R1,
                          const cv::Mat t1,
                          const cv::Mat R2,
                          const cv::Mat t2);


private:


    int min_num_inlier_;
    double ransac_confidence_;
    double essential_error_;
    double homography_error_;
    double r_proj_max_error_;
    double min_angle_;
    Specification type_;

    //TODO:三角量测的阈值

};



}

#endif //FEATURE_INIT_PAIR_H
