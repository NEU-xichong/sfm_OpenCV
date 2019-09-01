//
// Created by xc on 19-5-30.
//

#include "FeatureExtraction.h"
#include "Image.h"


namespace FeatureGraph{

void ScaleImage(cv::Mat src, cv::Mat& dst, const int& max_image_size, double& scale_x, double& scale_y);
void ExtractGoodFeature(std::vector<cv::KeyPoint> tem_KeyPoints,std::vector<cv::KeyPoint> & good_kpts,int topNum);



void 
FeatureExtraction::extractFeature(const cv::Mat &img, std::vector<cv::KeyPoint> &key_points, cv::Mat &des) {

    if(!img.data)
    {
        std::cerr<<"image do not exist!"<<std::endl;
    } else{

        cv::Mat scale_img;
        double scale_x;
        double scale_y;
        std::vector<cv::KeyPoint>scale_top_kpts;
        std::vector<cv::KeyPoint>temp_kpts;
        FeatureGraph::ScaleImage(img,scale_img,max_image_size_,scale_x,scale_y);

        cv::Ptr<cv::xfeatures2d::SIFT>ptr=cv::xfeatures2d::SIFT::create();

        ptr->detect(scale_img,temp_kpts);

        FeatureGraph::ExtractGoodFeature(temp_kpts,scale_top_kpts,max_num_features_);

        ptr->compute(scale_img,scale_top_kpts,des);

        key_points.resize(scale_top_kpts.size());

        const double inv_scale_x = 1.0 / scale_x;
        const double inv_scale_y = 1.0 / scale_y;
        const double inv_scale_xy = (inv_scale_x + inv_scale_y) / 2.0f;
        for(size_t i = 0; i < scale_top_kpts.size(); ++i)
        {
            key_points[i]=scale_top_kpts[i];
            key_points[i].pt.x = scale_top_kpts[i].pt.x * inv_scale_x;
            key_points[i].pt.y = scale_top_kpts[i].pt.y * inv_scale_y;
            key_points[i].size = scale_top_kpts[i].size * inv_scale_xy;
        }
    }

}

void 
ScaleImage(cv::Mat& src, cv::Mat& dst, const int& max_image_size, double& scale_x, double& scale_y)
{
    if(max_image_size < src.rows || max_image_size < src.cols)
    {
        const int width = src.cols;
        const int height = src.rows;
        const double scale = max_image_size * 1.0 / std::max(width, height);
        const int new_width = width * scale;
        const int new_height = height * scale;

        scale_x = new_width * 1.0 / width;
        scale_y = new_height * 1.0 / height;

        cv::resize(src, dst, cv::Size(new_width, new_height));
    }
    else
    {
        scale_x = 1.0;
        scale_y = 1.0;
        dst = src.clone();
    }
}
void 
ExtractGoodFeature(std::vector<cv::KeyPoint> tem_KeyPoints,std::vector<cv::KeyPoint> &good_kpts,int topNum)
{
    if(topNum>=tem_KeyPoints.size())
    {
        good_kpts=tem_KeyPoints;
    }
    else
    {
        std::vector<std::pair<size_t ,float >> response;
        for(int i=0;i<tem_KeyPoints.size();i++)
        {
            response.emplace_back(i,tem_KeyPoints[i].response);
        }

        std::partial_sort(response.begin(),response.begin()+topNum,
                            response.end(),[](const std::pair<size_t ,float >pair1,
                                            const std::pair<size_t ,float >pair2)
                                            { return pair1.second>pair2.second;}
                            );

        good_kpts.reserve(topNum);
        for(size_t i=0;i<topNum;i++)
        {
            good_kpts.push_back(tem_KeyPoints[response[i].first]);
        }
    }

}
}