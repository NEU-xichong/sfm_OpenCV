//
// Created by xc on 19-5-30.
//

#include "FeatureMatching.h"
namespace FeatureGraph
{
void CrossMatches(cv::Mat desc1,cv::Mat desc2,std::vector<cv::DMatch> matches,double distanceRatio,double maxDis);
void ComputeMatches(cv::Mat desc1,cv::Mat desc2,std::vector<cv::DMatch> matches,double distanceRatio,double maxDis);
void CrossCheck(std::vector<cv::DMatch> matches12,std::vector<cv::DMatch> matches21,std::vector<cv::DMatch> matches);



void 
FeatureMatching::matchFeatures(cv::Mat &desc1, cv::Mat &desc2, std::vector<cv::DMatch> &good_matches) {
    //std::vector<cv::DMatch> matches;
    FeatureGraph::CrossMatches(desc1,desc2,good_matches,distanceRatio_,max_distance_);

}

bool 
FeatureMatching::geometric_verif_F_H(const std::vector<cv::Point2f> &pts1,
                                            const std::vector<cv::Point2f> &pts2,
                                            const std::vector<cv::DMatch> matches,
                                            std::vector<cv::DMatch>&good_matches,
                                            H_F &H_F_CHOOSE,
                                            cv::Mat &Fundamental,
                                            cv::Mat &Homography) {

    //cv::Mat Fundamental;
    cv::Mat inlier_mask_F;
    cv::Mat inlier_mask_H;
    std::vector<cv::DMatch> matches_F;
    std::vector<cv::DMatch> matches_H;
    Fundamental=cv::findFundamentalMat(pts1,pts2,cv::FM_RANSAC,4.0,0.99,inlier_mask_F);
    Homography=cv::findHomography(pts1,pts2,cv::FM_RANSAC,12.0,inlier_mask_H,10000,0.99);


    assert(inlier_mask_F.type()==CV_8U);
    assert(inlier_mask_H.type()==CV_8U);


    size_t num_inlier_F=0;
    for(int i=0;i<inlier_mask_F.rows;i++)
    {
        if(inlier_mask_F.at<uchar>(i,0)==0)continue;
        matches_F.push_back(matches[i]);
        num_inlier_F++;
    }

    size_t num_inlier_H=0;
    for (int j = 0; j <inlier_mask_H.rows ; ++j) {

        if(inlier_mask_H.at<uchar>(j,0)==0)continue;
        matches_H.push_back(matches[j]);
        num_inlier_H++;

    }

    double H_F_ratio= static_cast<double>(num_inlier_H)/ static_cast<double >(num_inlier_F);

    //TODO:这个20没有经过实验测试，不一定可靠
    if(H_F_ratio<0.7&&num_inlier_F>=20)
    {
        H_F_CHOOSE=H_F::FundanmentalChoose;
        good_matches=matches_F;
        return true;

    }
    else if(H_F_ratio>=0.7&&num_inlier_H>=20)
    {
        H_F_CHOOSE=H_F::HomographyChoose;
        good_matches=matches_H;
        return true;
    }
    else
    {
        return false;

    }

}

void 
CrossMatches(cv::Mat desc1,cv::Mat desc2,std::vector<cv::DMatch> matches,double distanceRatio, double maxDis)
{
    std::vector<cv::DMatch> matches12;
    std::vector<cv::DMatch> matches21;

    ComputeMatches(desc1, desc2, matches12, distanceRatio,maxDis);
    ComputeMatches(desc2, desc1, matches21, distanceRatio,maxDis);

    CrossCheck(matches12, matches21, matches);

}


void 
ComputeMatches(cv::Mat desc1,cv::Mat desc2,std::vector<cv::DMatch> matches,double distanceRatio,double maxDis)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");

    std::vector<std::vector<cv::DMatch>> initial_matches;

    std::vector<cv::DMatch> middle_matches;



    matcher->knnMatch(desc1, desc2, initial_matches, 2);
    for(auto& match : initial_matches)
    {
        if(match[0].distance < distanceRatio * match[1].distance)
        {
            middle_matches.push_back(match[0]);
        }
    }

    double min_distance=std::numeric_limits<double>::max();
    double max_distance=std::numeric_limits<double>::min();

    for(int i=0;i<middle_matches.size();i++)
    {
        double dist=middle_matches[i].distance;
        if(dist<min_distance)
            min_distance=dist;
        if(dist>max_distance)
            max_distance=dist;
    }

    for(int i=0;i<middle_matches.size();i++)
    {
        //这里的200是经验值不一定准确
        if(middle_matches[i].distance< std::max(2*min_distance,maxDis))
        {
            matches.push_back(middle_matches[i]);
        }
    }

}
void 
CrossCheck(std::vector<cv::DMatch> matches12,std::vector<cv::DMatch> matches21,std::vector<cv::DMatch> matches)
{
    std::unordered_map<int, int> vis;


    for(size_t i = 0; i < matches21.size(); ++i)
    {
        int query_idx = matches21[i].queryIdx;
        int train_idx = matches21[i].trainIdx;
        vis[query_idx] = train_idx;
    }

    int good_matches = 0;
    for(size_t i = 0; i < matches12.size(); ++i)
    {

        int query_idx = matches12[i].queryIdx;
        int train_idx = matches12[i].trainIdx;

        if(vis[train_idx] == query_idx)
        {
            good_matches += 1;
            matches.push_back(matches12[i]);
        }
    }
}

}
