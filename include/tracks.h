//
// Created by xc on 19-6-3.
//

#ifndef FEATURE_TRACKS_H
#define FEATURE_TRACKS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "Image.h"

namespace FeatureGraph{


    struct View
    {
        View(int imgId,cv::Mat Image,std::vector<cv::KeyPoint> kpts):ImgId(imgId),img(Image),keyPoitsTrack(kpts){};

        int ImgId;

        cv::Mat img;

        //keyPoint对应了class_id;
        std::vector<cv::KeyPoint> keyPoitsTrack;
    };

    typedef std::vector<View> ViewList;


    struct FeatureIdOfView{

        FeatureIdOfView(int view_id,int feature_id):view_id_(view_id),feature_id_(feature_id){};

        int view_id_;

        int feature_id_;

    };

    typedef std::vector<FeatureIdOfView> FeatureIdOfViewList;


    struct Track{

        //每条track对应的世界坐标点
        cv::Vec3f pos;

        //每条tack对应世界坐标点的颜色
        cv::Vec3b color;

        //判断该track是否已经注册
        bool is_register=false;

        //map已经注册的3d的索引
        int p3d_idx=std::numeric_limits<int>::max();

        //没每条track包含的图像和图像上对应的特征点
        FeatureIdOfViewList featureTrack;

    };

    typedef std::vector<Track> TrackList;


    class tracks{

    public:

        tracks()= default;

        void computeTracks(const std::vector<pairImg> matching,ViewList &Views,TrackList &pointTracks);

        void unify_track(int view1_track_id,int view2_track_id,TrackList & trackAll,ViewList &viewports);

        void invalid_track_remove(ViewList Views,TrackList &pointTracks);

    };


}



#endif //FEATURE_TRACKS_H

