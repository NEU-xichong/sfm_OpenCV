//
// Created by xc on 19-6-3.
//
#include "init.h"
namespace FeatureGraph
{

/*
bool Initializer::RecoverPoseInitPair(FeatureGraph::pairImg &pair, std::vector<FeatureGraph::Camera> &Cam) {


    if(pair.choose==H_F::FundanmentalChoose)
    {
        return RecoverPoseFromE(pair,Cam);
    }

    return false;

    //TODO:如果全部不满足F，而是满足H,尽量通过F恢复，少用H恢复

}
*/
Return_Info_Init 
Initializer::RecoverPoseFromE(const std::vector<point2d2d> &matches,const cv::Mat& K) {

    if(type_==Specification::Unified)
    {
        cv::Mat E, R1, t1, R2, t2;
        cv::Mat inlier;

        //Camera &Cam1=Cam[pair.imgId1];
        //Camera &Cam2=Cam[pair.imgId2];

        std::vector<cv::Point2f> p2f1;
        std::vector<cv::Point2f> p2f2;

        for(size_t k=0;k<matches.size();++k)
        {
            p2f1.push_back(matches[k].p1);
            p2f2.push_back(matches[k].p2);
        }

        //这里使用Cam1或者Cam2的内参都行，因为是相等的
        E=cv::findEssentialMat(p2f1,p2f2,K,cv::RANSAC,ransac_confidence_,essential_error_,inlier);


        cv::recoverPose(E,p2f1,p2f2,K,R2,t2);

        //设置第一个相机为世界坐标
        R1=cv::Mat::eye(3,3,CV_64F);
        t1=cv::Mat::zeros(3,1,CV_64F);

        cv::Mat P1,P2;

        //其实这里相机1和2的内参矩阵是相等的
        cv::hconcat(K*R1,K*t1,P1);
        cv::hconcat(K*R2,K*t2,P2);


        std::vector<cv::Vec3d> points3D(matches.size());
        std::vector<double> tri_angles(matches.size(), 0);
        std::vector<double> residuals(matches.size(), std::numeric_limits<double>::max());
        std::vector<bool> inlier_mask(matches.size(), false);

        int num_liner=0;
        double sum_error=0.0;
        double sum_ang=0.0;

        std::vector<cv::Vec2d> point2D1=Utils::point2fToVec2ds(p2f1);
        std::vector<cv::Vec2d> point2D2=Utils::point2fToVec2ds(p2f2);

        for (int i = 0; i <point2D1.size() ; ++i) {

            if(inlier.at<uchar>(i,0)==0)
                continue;
            cv::Vec3d point3D=Triangulate(P1,P2,point2D1[i],point2D2[i]);

            bool positive_depth=PositiveDepth(point3D,R1,t1,R2,t2);
            double error=ReprojectPairError(point3D,point2D1[i],point2D2[i],R1,t1,R2,t2,K);
            double ang=CalculateAngle(point3D,R1,t1,R2,t2);


            points3D[i]=point3D;
            tri_angles[i]=ang;
            residuals[i]=error;

            if(positive_depth&& error<r_proj_max_error_)
            {
                num_liner++;
                inlier_mask[i]= true;
                sum_error+=error;
                sum_ang+=ang;

            }
            else{

                inlier_mask[i]= false;
            }

        }

        double ave_angle=sum_ang/num_liner;
        double ave_error=sum_error/num_liner;

        bool success=false;

        //TODO：这里内点数设置成100没有经过实验
        if(num_liner<min_num_inlier_||ave_angle<min_angle_)
        {
            success=false;
        }
        else
        {
            success=true;
        }

        Return_Info_Init return_info_;
        return_info_.is_succeed  =success;
        return_info_.num_inliers =num_liner;
        return_info_.ave_residual=ave_error;
        return_info_.R1=R1;
        return_info_.t1=t1;
        return_info_.R2=R2;
        return_info_.t2=t2;
        return_info_.points3D   =std::move(points3D);
        return_info_.tri_angles =std::move(tri_angles);
        return_info_.residuals  =std::move(residuals);
        return_info_.inlier_mask=std::move(inlier_mask);


        return return_info_;

    }

    //TODO：如果相机是不同内参的情况的处理
    else{


    }

}


    //TODO：从H矩阵中恢复相机位姿
void 
Initializer::RecoverPoseFromH(FeatureGraph::pairImg &pair, FeatureGraph::Camera &Cam) {


}


//两个点的三角量测
cv::Vec3d 
Initializer::Triangulate(const cv::Mat P1, const cv::Mat P2, const cv::Vec2d point1, const cv::Vec2d point2) {

    cv::Mat A(4, 4, CV_64F);
    // DLT
    A.row(0) = point1(0) * P1.row(2) - P1.row(0);
    A.row(1) = point1(1) * P1.row(2) - P1.row(1);
    A.row(2) = point2(0) * P2.row(2) - P2.row(0);
    A.row(3) = point2(1) * P2.row(2) - P2.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u ,vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    // 最小特征值所对应的特征向量
    cv::Mat point3D = vt.row(3).t();

    assert(point3D.type() == CV_64F);

    // 从齐次坐标 -> 非齐次坐标
    point3D = point3D.rowRange(0, 3) / point3D.at<double>(3, 0);

    double x = point3D.at<double>(0);
    double y = point3D.at<double>(1);
    double z = point3D.at<double>(2);
    return cv::Vec3d(x, y, z);
}

//多个点的三角量测
Return_Info_Tri
Initializer::Triangulate(const std::vector<cv::Mat>& Rs,
                         const std::vector<cv::Mat>& ts,
                         const std::vector<cv::Vec2d>& points2D,
                         const cv::Mat &K_)
{

    Return_Info_Tri return_info_;
    assert(Rs.size() != 0);
    assert(Rs.size() == ts.size());
    assert(Rs.size() == points2D.size());

    cv::Vec3d point3d=TriangulateMultiviewPoint(Rs,ts,points2D,K_);

    return_info_.point3D=point3d;

    double sum_residual=0;
    double num_inliers=0;

    return_info_.inliers_=std::vector<bool>(points2D.size(),false);

    for(size_t i = 0; i < points2D.size(); ++i)
    {
        double error = CalculateReprojectError(point3d, points2D[i], Rs[i], ts[i], K_);
        //residuals[i] = error;
        if(error > r_proj_max_error_)
        {
            //break;
            continue;
        }
        sum_residual += error;
        num_inliers += 1;
        return_info_.inliers_[i]=true; //为true的加入到map里去
    }

    return_info_.ave_residual=sum_residual/num_inliers;
    return_info_.is_succeed=true;

    return return_info_;
}


cv::Vec3d
Initializer::TriangulateMultiviewPoint(const std::vector<cv::Mat>& Rs,
                                       const std::vector<cv::Mat>& ts,
                                       const std::vector<cv::Vec2d>& points2D,
                                       const cv::Mat &K_)
{
    cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
    for(size_t i = 0; i < points2D.size(); ++i)
    {
        cv::Mat proj_matrix;
        cv::hconcat(K_ * Rs[i], K_ * ts[i], proj_matrix);

        const cv::Mat term1 = points2D[i](0) * proj_matrix.row(2) - proj_matrix.row(0);
        const cv::Mat term2 = points2D[i](1) * proj_matrix.row(2) - proj_matrix.row(1);

        A += term1.t() * term1;
        A += term2.t() * term2;
    }

    cv::Mat eigenvalues;
    cv::Mat eigenvector;
    cv::eigen(A, eigenvalues, eigenvector);

    assert(eigenvector.type() == CV_64F);

    double x = eigenvector.at<double>(3, 0) / eigenvector.at<double>(3, 3);
    double y = eigenvector.at<double>(3, 1) / eigenvector.at<double>(3, 3);
    double z = eigenvector.at<double>(3, 2) / eigenvector.at<double>(3, 3);

    cv::Vec3d point3D(x, y, z);

    return point3D;
}

bool 
Initializer::PositiveDepth(const cv::Vec3d point3d, const cv::Mat R1, const cv::Mat t1, const cv::Mat R2,
                                const cv::Mat t2) {

    cv::Mat Point3D=cv::Mat::zeros(4,1,CV_64F);
    Point3D.at<double>(0,0)=point3d(0);
    Point3D.at<double>(1,0)=point3d(1);
    Point3D.at<double>(2,0)=point3d(2);
    Point3D.at<double>(3,0)=1;

    cv::Mat cam1_matrix;
    cv::Mat cam2_matrix;

    cv::hconcat(R1,t1,cam1_matrix);
    cv::hconcat(R2,t2,cam2_matrix);

    cv::Mat tran_point1;
    cv::Mat tran_point2;

    tran_point1=cam1_matrix*Point3D;
    tran_point2=cam2_matrix*Point3D;

    if(tran_point1.at<double>(2,0)>std::numeric_limits<double>::epsilon()&&
       tran_point2.at<double>(2,0)>std::numeric_limits<double>::epsilon())
    {
        return true;
    }
    else
    {
        return false;
    }
}

double 
Initializer::ReprojectPairError(const cv::Vec3d point3d, const cv::Vec2d point2d1, const cv::Vec2d point2d2,
                                   const cv::Mat R1, const cv::Mat t1, const cv::Mat R2, const cv::Mat t2,
                                   const cv::Mat K) {
    double cam1_error=CalculateReprojectError(point3d,point2d1,R1,t1,K);
    double cam2_error=CalculateReprojectError(point3d,point2d2,R2,t2,K);

    return (cam1_error+cam2_error)/2;
}
double 
Initializer::CalculateReprojectError(const cv::Vec3d point3d, const cv::Vec2d point2d, const cv::Mat R,
                                            const cv::Mat t, const cv::Mat K) {

    cv::Mat project_matrix;
    cv::hconcat(K*R,K*t,project_matrix);


    cv::Mat Point3D=cv::Mat::zeros(4,1,CV_64F);
    Point3D.at<double>(0,0)=point3d(0);
    Point3D.at<double>(1,0)=point3d(1);
    Point3D.at<double>(2,0)=point3d(2);
    Point3D.at<double>(3,0)=1;

    cv::Mat proj_2d=project_matrix*Point3D;

    proj_2d/=proj_2d.at<double>(2,0);

    double x_error=proj_2d.at<double>(0,0)-point2d(0);
    double y_error=proj_2d.at<double>(1,0)-point2d(1);

    double error=std::sqrt(x_error*x_error+y_error*y_error);
    return error;

}


double 
Initializer::CalculateAngle(const cv::Vec3d point3d, const cv::Mat R1, const cv::Mat t1, const cv::Mat R2,
                                   const cv::Mat t2) {

    cv::Mat Cam1Centra;
    cv::Mat Cam2Centra;

    //相机的中心为R.t()*t,即旋转的转置乘以平移
    Cam1Centra=-R1.t()*t1;
    Cam2Centra=-R2.t()*t2;

    cv::Vec3d center1(Cam1Centra.at<double>(0,0),Cam1Centra.at<double>(1,0),Cam1Centra.at<double>(2,0));
    cv::Vec3d center2(Cam2Centra.at<double>(0,0),Cam2Centra.at<double>(1,0),Cam2Centra.at<double>(2,0));


    cv::Vec3d ray1=center1-point3d;
    cv::Vec3d ray2=center2-point3d;

    cv::Vec3d baseLine=center2-center1;

    double ray1lenth=cv::norm(ray1);
    double ray2lenth=cv::norm(ray2);
    double baselineLenth=cv::norm(baseLine);

    double angle=std::abs(std::acos(
            (ray1lenth*ray1lenth+ray2lenth*ray2lenth-baselineLenth*baselineLenth)/(2*ray1lenth*ray2lenth)));

    return std::min<double>(angle,M_PI-angle)*180/M_PI;


}





}