#ifndef FEATURE_SFM_H
#define FEATURE_SFM_H
#include<opencv2/opencv.hpp>
#include<string>

#include"Image.h"
#include"init.h"
#include"map.h"
#include"PnP3D2D.h"
#include"tracks.h"
#include"BundelOptimizer.h"
#include"easy_IO_exif.h"
#include"common.h"


//#define SORT_TEST


namespace FeatureGraph
{

class Increment_SFM
{
public:
    


    Increment_SFM(std::string &img_path_,std::string &vocab_path_,Params paramters_,Camera_Parameters para,std::string& para_path_);

    //成员变量的初始化
    bool Init();


    //图像特征提取和匹配
    void Extraction_Matching();


    //构建track
    void Create_tracks();


    //计算候选对，从候选对中选出初始对
    void compute_candidate_pair();

    //初始对的重建
    void Initialize();

    //查找下一个重建的图像
    void find_next_img();

    //SFM
    void Sfm();


    void Get2d3d_match(const int&img_id,
                       std::vector<cv::Vec2f> &point2ds,
                       std::vector<cv::Vec3f> &point3ds,
                       std::vector<int>       &point2d_idxs,
                       std::vector<int>       &point3d_idxs      
    );

    void Triangulate_new_tracks();

private:

    //重建所需要的一些类的成员变量
    std::shared_ptr<Image>                  Images_;
    std::shared_ptr<Initializer>            Init_;
    std::shared_ptr<Bundel_Opt>             Bundel_;
    std::shared_ptr<tracks>                 Tracks_;
    std::shared_ptr<PnP3D2D>                PnpSlover_;
    std::shared_ptr<Map>                    Maps_;
    //std::shared_ptr<exifExtract>                


    //track中需要的参数
    ViewList views;
    TrackList tracklist_;




    //一些类变量初始化需要的参数
    std::string img_path; 
    std::string vocab_path;

    //相机参数路径（只要当参数类型是用户提供时候才有效）
    std::string para_path;

    //用与判断相机参数是从exif中提取，还是用户自己给定
    Camera_Parameters parameter;

    //一些函数所需的参数，有默认值，当让用户也可以更改
    Params paramters;



    //初始化初始对的时候需要
    CandidatePairs CandidatePairs_; 


    //下一个要重建的图像    
    std::vector<int>next_imgs;
    
    //三角量测需要的最少图像个数
    int min_views_num=2;    //就地初始化，对所有的构造函数的值都一样

    //相机参数，目前仅支持同一规格的相机拍摄的图片
    cv::Mat K_;
    cv::Mat dist_coef_;

};


}





#endif