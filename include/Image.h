//
// Created by xc on 19-5-30.
//

#ifndef FEATUREMATCH_IMAGE_H
#define FEATUREMATCH_IMAGE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <math.h>
#include "DBoW3/DBoW3.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include "FeatureExtraction.h"
#include "FeatureMatching.h"
#include "common.h"
#include "Camera.h"
using namespace boost::filesystem;



namespace FeatureGraph
{


struct Img
{
    Img()= default;
    Img(const std::string & path,int id):path_(path),id_(id){};
    cv::Mat imgRead(const std::string & path);
    int id_;
    std::string path_;
    cv::Mat img_;
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptor;
};



struct pairImg{

     int imgId1;
     int imgId2;

     std::vector<cv::Point2f> points2f1;
     std::vector<cv::Point2f> points2f2;

     std::vector<cv::DMatch> matches;

     //TODO：这个应该放在初始化里，好理解些
     //void computeE(std::vector<Camera> &Cam ,const Parameter &parameter=Parameter::Unified);
     H_F choose;

     cv::Mat F;
     cv::Mat E;
     cv::Mat H;
     bool operator==(const struct pairImg &rhs)
     {
         return ((imgId1==rhs.imgId1)&&(imgId2==rhs.imgId2))||((imgId2==rhs.imgId1)&&(imgId1==rhs.imgId2));
     }
};

class Image{


public:


    // void imgRead (const std::string &path);

    Image(const cv::Ptr<FeatureExtraction> Extr = cv::Ptr<FeatureExtraction>(new FeatureExtraction()),
          const cv::Ptr<FeatureMatching> Match = cv::Ptr<FeatureMatching>(new FeatureMatching())
                  ):featureExtr(Extr),featureMatch(Match){};

    bool ExistImgId(const int id) const;

    int loadImg(const std::string & path);

    cv::Mat GetImgById(const int id) const;

    std::vector<cv::KeyPoint> GetKeypotsById(const int id)const;

    cv::Mat GetDesById(const int id) const;

    int createVocabSim(const std::string &vocabPath,
                       const size_t numSim
            );

    void Homography_Fundamental_geometric_verif();

    int createResMatch(const std::map<int,std::vector<int>> simGrap,std::map<int,std::vector<int>> resGraph);




    //成员变量



public:


    std::vector <Img> ImgVec;

    struct pairImg pairImage;

    std::vector<pairImg> pairVec;

    cv::Ptr<FeatureExtraction> featureExtr ;//= cv::Ptr<FeatureExtraction>(new FeatureExtraction());

    cv::Ptr<FeatureMatching> featureMatch ;

    //由字典书得到的关系图
    std::map<int,std::vector<int>> vocabSimilar;

    //由单应矩阵过滤后的关系图
    std::map<int,std::vector<int>> GraphAfterGeometric_verif;

    //最终的关系图
    std::map<int,std::vector<int>> resMatch;


};

}





#endif //FEATUREMATCH_IMAGE_H
