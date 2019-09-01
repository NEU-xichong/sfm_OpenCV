//
// Created by xc on 19-5-30.
//

#include "Image.h"
#include <opencv2/opencv.hpp>
#include <cctype>


namespace FeatureGraph
{

typedef struct tempPair{
    size_t idImg;

    float inlerRatio;

    std::vector<cv::DMatch> tempMatches;

    bool operator<(const tempPair & rhs)const
    {
        return inlerRatio<rhs.inlerRatio;
    }

} tempPair;

cv::Mat 
Img::imgRead(const std::string &path) {

    std::string str4=path.substr(path.size()-4);
    std::string str5=path.substr(path.size()-5);

    std::transform(str4.begin(),str4.end(),str4.begin(),tolower);
    std::transform(str5.begin(),str5.end(),str5.begin(),tolower);

    cv::Mat img;

    if(str4 == ".jpg" or str4 == ".png" or str5 ==".jpeg"  )
    {
        img=cv::imread(path);
    }
    else{
        std::cerr<<"could not read img file!"<<std::endl;
        return cv::Mat();
    }
    //img_=cv::imread(path);
    return img;
}

//TODO::这个应该放在init里
/*
void pairImg::computeE( std::vector <Camera> &Cam, const Parameter &parameter) {


    Camera &camera1=Cam[imgId1];

    Camera &camera2=Cam[imgId2];

    if(parameter==Parameter::Unified)
    {


    }
}
*/
int 
Image::loadImg(const std::string &imgPath) {

    path dirPath(imgPath);
    if(not exists(dirPath) or not is_directory(dirPath))
    {
        std::cerr<<"File cannot be opened!"<<std::endl;
        return -1;
    }

    int count=0;

    //cv::Ptr<FeatureExtraction> featureExtr = cv::Ptr<FeatureExtraction>(new FeatureExtraction());

    for(directory_entry& imgPath:directory_iterator(dirPath))
    {
        std::cout<<"image: "<<count<<'\n'<<"imgDir: "<<imgPath.path().string()<<std::endl;

        ImgVec.push_back(Img(imgPath.path().string(),count));

        ImgVec[count].img_=ImgVec[count].imgRead(imgPath.path().string());

        //FeatureExtraction *featureExtr=new FeatureExtraction();

        featureExtr->extractFeature(ImgVec[count].img_,ImgVec[count].keyPoints,ImgVec[count].descriptor);

        count++;
    }

}

bool 
Image::ExistImgId(const int id) const {

    if(!ImgVec[id].img_.data)
    {
        return false;
    }
    else {
        return true;
    }

}

cv::Mat 
Image::GetImgById(const int id) const {

    return ImgVec[id].img_;
}

std::vector<cv::KeyPoint> 
Image::GetKeypotsById(const int id) const {

    return ImgVec[id].keyPoints;
}

cv::Mat 
Image::GetDesById(const int id) const {

    return ImgVec[id].descriptor;
}


int 
Image::createVocabSim(const std::string &vocabPath,
                            const size_t numSim) {

    std::cout<<"comparing images with database "<<std::endl;

    DBoW3::Vocabulary vocab(vocabPath);
    if(vocab.empty())
    {
        std::cerr<<"Vocabulary dose not exist."<<std::endl;
        return 0;
    }

    DBoW3::Database db(vocab, false,0);

    for(int i=0;i<ImgVec.size();i++)
    {
        db.add(ImgVec[i].descriptor);
    }

    for(int i=0;i<ImgVec.size();i++)
    {
        DBoW3::QueryResults ret;

        //TODO：这个最大相似的数量取多少，有待实验确定，默认选6个，在从中间选4个

        db.query(ImgVec[i].descriptor,ret,numSim);
        //因为j=0是本身
        for(int j=1;j<ret.size();j++)
        {
            vocabSimilar[i].push_back(ret[j].Id);
        }
    }

    return 1;

}

/**
 * 从字典树得出的每一副图像与自身相似的前6个图像，再从前六副图像中选择单应矩阵内点数前4小的四副图像
 */
void 
Image::Homography_Fundamental_geometric_verif() {


    //std::vector<struct tempPair> imgTempPairVec;
    std::map<int,std::vector<tempPair>>  imgTempPairMap;


    for(int i=0;i<vocabSimilar.size();i++)
    {
        for(int j=0;j<vocabSimilar[i].size();j++)
        {
            tempPair imgTempPair;

            int id=vocabSimilar[i][j];
            imgTempPair.idImg=id;

            featureMatch->matchFeatures(ImgVec[i].descriptor,ImgVec[id].descriptor,imgTempPair.tempMatches);

            //TODO:计算单应矩阵的内点数
            std::vector<cv::Point2f> pts1;
            std::vector<cv::Point2f> pts2;

            for(int x=0;x<(int) imgTempPair.tempMatches.size();i++)
            {
                pts1.push_back(ImgVec[i].keyPoints[imgTempPair.tempMatches[x].queryIdx].pt);
                pts2.push_back(ImgVec[id].keyPoints[imgTempPair.tempMatches[x].trainIdx].pt);
            }

            cv::Mat homography;
            cv::Mat inlierMask;
            if(imgTempPair.tempMatches.size()>=4)
            {
                homography=cv::findHomography(pts1,pts2,cv::RANSAC,10,inlierMask);
            }

            if(imgTempPair.tempMatches.size()<4||homography.empty())
            {
                imgTempPair.inlerRatio=std::numeric_limits<float >::max();
            }
            else{
                imgTempPair.inlerRatio=(float) cv::countNonZero(inlierMask)/(float)imgTempPair.tempMatches.size();
            }
            imgTempPairMap[i].push_back(imgTempPair);
        }
        //将单应矩阵的内点比率排序，内点比率越小，越优先考虑
        std::sort(imgTempPairMap[i].begin(),imgTempPairMap[i].end());


        //基础矩阵验证
        for(int k=0;k<4;k++)
        {
            std::vector<cv::Point2f> points1;
            std::vector<cv::Point2f> points2;

            int idF=imgTempPairMap[i][k].idImg;
            cv::Mat F;
            cv::Mat H;
            H_F h_f;
            bool isSatisfyF= false;
            std::vector<cv::DMatch> finalMatch;

            for(int x=0;x<(int) imgTempPairMap[i][k].tempMatches.size();i++)
            {
                points1.push_back(ImgVec[i].keyPoints[imgTempPairMap[i][k].tempMatches[x].queryIdx].pt);
                points2.push_back(ImgVec[idF].keyPoints[imgTempPairMap[i][k].tempMatches[x].trainIdx].pt);
            }

            isSatisfyF=featureMatch->geometric_verif_F_H(points1,points2,imgTempPairMap[i][k].tempMatches,finalMatch,h_f,F,H);
            if(isSatisfyF== true)
            {
                pairImage.imgId1=i;
                pairImage.imgId2=idF;
                pairImage.F=F;
                pairImage.H=H;
                pairImage.choose=h_f;
                pairImage.matches=finalMatch;

                for(size_t s=0;s<finalMatch.size();++s)
                {
                    pairImage.points2f1.push_back(ImgVec[i].keyPoints[finalMatch[s].queryIdx].pt);
                    pairImage.points2f2.push_back(ImgVec[idF].keyPoints[finalMatch[s].trainIdx].pt);
                }

                if(std::count(pairVec.begin(),pairVec.end(),pairImage)==0)
                {
                    pairVec.push_back(pairImage);
                }
                GraphAfterGeometric_verif[i].push_back(imgTempPairMap[i][k].idImg);
            }
        }
    }
}

int 
Image::createResMatch(const std::map<int, std::vector<int>> simGrap,
                            std::map<int, std::vector<int>> resGraph) {

}

}