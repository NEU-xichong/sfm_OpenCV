//
// Created by xc on 19-6-8.
//
#include "BundelOptimizer.h"


namespace FeatureGraph
{


//优化参数为相机的位姿
struct BundleAutoDiffConstantFocalCostFunction
{
    BundleAutoDiffConstantFocalCostFunction(double observed_x,
                                            double observed_y,
                                            double focal_x,
                                            double focal_y) :
        observed_x(observed_x), observed_y(observed_y), focal_x(focal_x), focal_y(focal_y){}

    template<typename T>
    bool operator()(const T* const rvec,
                    const T* const tvec,
                    const T* const point,
                          T* residuals) const
    {
        T p[3];
        //对点point施加rvec所对应的旋转，　结果存储在p中
        ceres::AngleAxisRotatePoint(rvec, point, p);
        //加上平移
        p[0] += tvec[0];
        p[1] += tvec[1];
        p[2] += tvec[2];

        //齐次坐标归一化
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        const T predicted_x = T(focal_x) * xp;
        const T predicted_y = T(focal_y) * yp;

        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }


    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y,
                                       const double focal_x,
                                       const double focal_y)
    {
        //2 表示残差项的维度为2
        //3	表示rvec的维度为3
        //3	表示tvec的维度为3
        //3 表示point的维度为3
        return (new ceres::AutoDiffCostFunction<BundleAutoDiffConstantFocalCostFunction, 2, 3, 3, 3>(
                new BundleAutoDiffConstantFocalCostFunction(observed_x, observed_y, focal_x, focal_y)));
    }
    double observed_x;
    double observed_y;
    double focal_x;
    double focal_y;
};


//优化参数为相机的位姿，三维点以及相机的焦距，这一部分优化主要是针对焦距是从exif中提取出来的情况
struct BundleAutoDiffCostFunction
{
    BundleAutoDiffCostFunction(double observed_x, double observed_y) :
        observed_x(observed_x), observed_y(observed_y){}

    template<typename T>
    bool operator()(const T* const rvec,
                    const T* const tvec,
                    const T* const point,
                    const T* const focal,
                          T* residuals) const
    {
        T p[3];
        //对点point施加rvec所对应的旋转，　结果存储在p中
        ceres::AngleAxisRotatePoint(rvec, point, p);
        //加上平移
        p[0] += tvec[0];
        p[1] += tvec[1];
        p[2] += tvec[2];

        //齐次坐标归一化
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        const T predicted_x = focal[0] * xp;
        const T predicted_y = focal[1] * yp;

        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }


    static ceres::CostFunction* Create(const double observed_x, const double observed_y)
    {
        //2 表示残差项的维度为2
        //3	表示rvec的维度为3
        //3	表示tvec的维度为3
        //3 表示point的维度为3
        //2 表示focal的维度为2
        return (new ceres::AutoDiffCostFunction<BundleAutoDiffCostFunction, 2, 3, 3, 3, 2>(
                new BundleAutoDiffCostFunction(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};






}