//
// Created by xc on 19-6-8.
//
#include "PnP3D2D.h"


namespace FeatureGraph{


Return_Info_P3d2d 
PnP3D2D::PnP3d2dSolve(std::vector<cv::Point3f> point3ds,std::vector<cv::Point2f> point2ds,cv::Mat K_)
{
    assert(point2ds.size()==point3ds.size());

    Return_Info_P3d2d return_info;

    return_info.num_point2D_3D_correspondences=point3ds.size();

    if(point2ds.size()<inliers_num)
    {
        return return_info;
    }

    cv::Mat R_;
    cv::Mat t_;
    cv::Mat inlier_;


    switch (PnPMethod)
    {
    case P3P:
        cv::solvePnPRansac(point3ds,point2ds,K_,cv::Mat(),R_,t_,false,
                           iterative_opt,max_error,ransac_confidence,inlier_,cv::SOLVEPNP_P3P);
        break;
    case AP3P:
        cv::solvePnPRansac(point3ds,point2ds,K_,cv::Mat(),R_,t_,false,
                           iterative_opt,max_error,ransac_confidence,inlier_,cv::SOLVEPNP_AP3P);
        break;
    case EPNP:
        cv::solvePnPRansac(point3ds,point2ds,K_,cv::Mat(),R_,t_,false,
                           iterative_opt,max_error,ransac_confidence,inlier_,cv::SOLVEPNP_EPNP);
        break;
    case UPNP:
        cv::solvePnPRansac(point3ds,point2ds,K_,cv::Mat(),R_,t_,false,
                           iterative_opt,max_error,ransac_confidence,inlier_,cv::SOLVEPNP_UPNP);
        break;
    default:
        break;
    }

    if(inlier_.rows<inliers_num)
    {
        return return_info;
    }

    std::vector<bool> inlier_mask(point3ds.size(), false);

    for(int i = 0; i < inlier_.rows; ++i)
    {
        int idx = inlier_.at<int>(i, 0);
        inlier_mask[idx] = true;
    }


    std::vector<double> residuals(inlier_mask.size());
    for(size_t i = 0; i < inlier_mask.size(); ++i)
    {
        double error = Initializer::CalculateReprojectError(Utils::point3fToVec3d(point3ds[i]),Utils::point2fToVec2d(point2ds[i]), R_, t_, K_);
        if(!inlier_mask[i])
            continue;
        residuals[i] = error;

    }

    cv::Rodrigues(R_,return_info.R);
    return_info.t=t_.clone();

    return_info.is_success = true;
    return_info.num_inliers = static_cast<size_t>(inlier_.rows);

    return_info.residuals = std::move(residuals);
    return_info.inlier_mask = std::move(inlier_mask);


    return return_info;
    

    
}





}