//
// Created by xc on 19-6-3.
//
#include "tracks.h"

namespace FeatureGraph
{

void delete_track_clean(std::vector<bool> &delete_track,TrackList &track);

void 
tracks::unify_track(int view1_track_id,int view2_track_id, FeatureGraph::TrackList &trackAll,
                         FeatureGraph::ViewList &viewports) {

    //将track特征数量少的，融合到track特征数量多的
     if(trackAll[view1_track_id].featureTrack.size()<trackAll[view2_track_id].featureTrack.size())
     {
         std::swap(view1_track_id,view2_track_id);
     }

     Track& track1=trackAll[view1_track_id];
     Track& track2=trackAll[view2_track_id];

     //将原本track2中的特征，换成track1
     for(size_t k=0;k<track2.featureTrack.size();++k)
     {

         int view_id=track2.featureTrack[k].view_id_;
         int feature_id=track2.featureTrack[k].feature_id_;

         viewports[view_id].keyPoitsTrack[feature_id].class_id=view1_track_id;

     }

     track1.featureTrack.insert(track1.featureTrack.end(),track2.featureTrack.begin(),track2.featureTrack.end());

     //释放空间
     track2.featureTrack=FeatureIdOfViewList();

}


void 
tracks::computeTracks(const std::vector<FeatureGraph::pairImg> matching, FeatureGraph::ViewList &Views,
                           FeatureGraph::TrackList &pointTracks) {


    std::cout<<"begin construct tracks....."<<std::endl;

    pointTracks.clear();

    for(size_t i=0;i<matching.size();i++)
    {

        pairImg pairwise=matching[i];

        View &view1=Views[pairwise.imgId1];
        View &view2=Views[pairwise.imgId2];

        for(size_t j=0;j<pairwise.matches.size();j++)
        {
            int view1_track_id=view1.keyPoitsTrack[pairwise.matches[j].queryIdx].class_id;

            int view2_track_id=view2.keyPoitsTrack[pairwise.matches[j].trainIdx].class_id;

            if(view1_track_id==-1 && view2_track_id==-1)
            {

                view1.keyPoitsTrack[pairwise.matches[j].queryIdx].class_id=pointTracks.size();
                view2.keyPoitsTrack[pairwise.matches[j].trainIdx].class_id=pointTracks.size();
                pointTracks.push_back(Track());

                pointTracks.back().featureTrack.push_back(FeatureIdOfView(pairwise.imgId1,pairwise.matches[j].queryIdx));
                pointTracks.back().featureTrack.push_back(FeatureIdOfView(pairwise.imgId2,pairwise.matches[j].trainIdx));

            }
            else if(view1_track_id==-1 && view2_track_id!=-1)
            {
                view1.keyPoitsTrack[pairwise.matches[j].queryIdx].class_id=view2_track_id;

                pointTracks[view2_track_id].featureTrack.push_back(FeatureIdOfView(pairwise.imgId1,pairwise.matches[j].queryIdx));


            }
            else if(view1_track_id!=-1 && view2_track_id ==-1)
            {
                view2.keyPoitsTrack[pairwise.matches[j].trainIdx].class_id=view1_track_id;
                pointTracks[view1_track_id].featureTrack.push_back(FeatureIdOfView(pairwise.imgId2,pairwise.matches[j].trainIdx));

            }

            else if(view1_track_id==view2_track_id)
            {
                //当两个track的id相等时，则不用计算
            }
            else{
                unify_track(view1_track_id,view2_track_id,pointTracks,Views);
            }
        }
    }

    invalid_track_remove(Views,pointTracks);



}

void 
tracks::invalid_track_remove(FeatureGraph::ViewList Views, FeatureGraph::TrackList &pointTracks) {


    std::vector<bool> delete_tracks(pointTracks.size());

    for(size_t i=0;i<pointTracks.size();++i)
    {
        if(pointTracks[i].featureTrack.empty())
        {
            delete_tracks[i]=true;
            continue;
        }

        std::set<int> view_ids;

        //一条track里有同一图像里的不同特征点，则删除该track
        for(size_t j=0;j<pointTracks[i].featureTrack.size();++j)
        {
            FeatureIdOfView  const& ref=pointTracks[i].featureTrack[j];
            if(view_ids.insert(ref.view_id_).second== false)
            {
                delete_tracks[i]=true;
                break;
            }
        }
    }


    //TODO：创建从老的track ID到新的track ID
    std::vector<int> new_map(delete_tracks.size(),-1);

    int track_count=0;

    for(size_t i=0;i<delete_tracks.size();i++)
    {
        if(delete_tracks[i]= true)
            continue;
        new_map[i]=track_count;
        track_count++;
    }

    //将原来图像feature里的对应的track改成新生成的track（排除错误track以后的track）
    for(size_t i=0;i<Views.size();++i)
    {
        for(size_t j=0;j<Views[i].keyPoitsTrack.size();++j)
        {
            if(Views[i].keyPoitsTrack[j].class_id>=0)
                Views[i].keyPoitsTrack[j].class_id=new_map[Views[i].keyPoitsTrack[j].class_id];
        }
    }

    //pointTrack中删除已经去除掉的track
    delete_track_clean(delete_tracks,pointTracks);

    for(size_t i=0;i<pointTracks.size();++i)
    {
        Track & trackColor=pointTracks[i];
        int numFeature=0;
        cv::Vec3b color(0,0,0);
        for(int j=0;j<trackColor.featureTrack.size();++j)
        {
            FeatureIdOfViewList &ref=trackColor.featureTrack;

            View viewColor=Views[ref[j].view_id_];

            int row=viewColor.keyPoitsTrack[ref[j].feature_id_].pt.y;
            int col=viewColor.keyPoitsTrack[ref[j].feature_id_].pt.x;

            //TODO:这一部分的类型可能存在问题！！！！！
            color+=cv::Vec3b(viewColor.img.at<cv::Vec3b>(row,col)[0],viewColor.img.at<cv::Vec3b>(row,col)[1],viewColor.img.at<cv::Vec3b>(row,col)[2]);
            numFeature++;
        }
        trackColor.color= static_cast<cv::Vec3b>(color/numFeature);
    }

}


void 
delete_track_clean(std::vector<bool> &delete_track,TrackList &track)
{
    std::vector<Track>::iterator vr=track.begin();
    std::vector<Track>::iterator vw=track.begin();

    std::vector<bool>::const_iterator dr=delete_track.begin();

    while (vr!=track.end()&& dr!=delete_track.end()){

        if(*dr++)
        {
            vr++;
            continue;
        }
        if(vw!=vr)
            *vw=*vr;
        vw++;
        vr++;
    }
    track.erase(vw,track.end());
}



}
