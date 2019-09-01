#include"sfm.h"

namespace FeatureGraph
{

Increment_SFM::Increment_SFM(std::string &img_path_,std::string &vocab_path_,Params paramters_,Camera_Parameters para,std::string& para_path_):
                            img_path(img_path_),vocab_path(vocab_path_),paramters(paramters_),parameter(para),para_path(para_path_)
{
    //dist_coef_=(cv::Mat_<double>(4, 1) <<0, 0, 0, 0);

    if(para==Camera_Parameters::Exsit_Para)
    {
        cv::FileStorage file_(para_path,cv::FileStorage::READ);
        if(!file_.isOpened())
        {
            //printf("%s %s %s\n","YAML file: ",para_path," open fail!");
            std::cerr<<"YAML file: "<<para_path<<" open fail!"<<std::endl;
            exit(-1);
        }
        file_["Reconstruction.Camera.fx"] >> paramters.fx;
        file_["Reconstruction.Camera.fy"] >> paramters.fy;
        file_["Reconstruction.Camera.cx"] >> paramters.cx;
        file_["Reconstruction.Camera.cy"] >> paramters.cy;

        file_["Reconstruction.Camera.k1"] >> paramters.k1;
        file_["Reconstruction.Camera.k2"] >> paramters.k2;
        file_["Reconstruction.Camera.p1"] >> paramters.p1;
        file_["Reconstruction.Camera.p2"] >> paramters.p2;

        K_ = (cv::Mat_<double>(3, 3) <<paramters.fx, 0,paramters.cx,
                                       0,paramters.fy,paramters.cy,
                                       0, 0, 1);
        dist_coef_ = (cv::Mat_<double>(4, 1) <<paramters.k1,paramters.k2,paramters.p1,paramters.p2);
    }

    //TODO:从exif中读取参数
    else
    {
        
    }
    

}

bool
Increment_SFM::Init()
{
    Images_   =std::make_shared<Image>(Image());
    Init_     =std::make_shared<Initializer>(Initializer(paramters.Init_para_.min_angle,paramters.Init_para_.ransac_confidence,
                                                             paramters.Init_para_.essential_error,paramters.Init_para_.homography_error,
                                                             paramters.Init_para_.r_proj_max_error,paramters.Init_para_.min_angle,
                                                             paramters.Init_para_.typeImg_Attr));
    Bundel_   =std::make_shared<Bundel_Opt>(Bundel_Opt());
    Tracks_   =std::make_shared<tracks>(tracks());
    PnpSlover_=std::make_shared<PnP3D2D>(PnP3D2D(paramters.PnP3D2D_para_.PnPMethod_,paramters.PnP3D2D_para_.ransac_confidence_,
                                                     paramters.PnP3D2D_para_.max_error_,paramters.PnP3D2D_para_.iterative_opt_,
                                                     paramters.PnP3D2D_para_.inliers_num_));
    Maps_     =std::make_shared<Map>(Map());

}


void
Increment_SFM::Extraction_Matching()
{   
    //加载了图像，同时计算的图像的特征点和特征描述子
    Images_->loadImg(img_path);

    //TODO:这里暂时默认为6，可以根据实际情况修改
    Images_->createVocabSim(vocab_path,6);

    //计算了图像之间的匹配关系
    Images_->Homography_Fundamental_geometric_verif();

}


void
Increment_SFM::Create_tracks()
{

    //先构造views
    for(size_t i=0;i<Images_->ImgVec.size();++i)
    {
        views.emplace_back(View(Images_->ImgVec[i].id_,Images_->ImgVec[i].img_,Images_->ImgVec[i].keyPoints));
    }
    
    //根据匹配关系生成track
    Tracks_->computeTracks(Images_->pairVec,views,tracklist_);

}


/*计算初始对，按照图像之间的匹配数量进行排序，得到一个匹配对数组，按照匹配对的数量降序排序*/
void
Increment_SFM::compute_candidate_pair()
{
    int num_views=static_cast<int>(views.size());

    std::vector<int>candidate_lookup(num_views*num_views,-1);

    for(int i=0;i<tracklist_.size();++i)
    {
        const Track& track=tracklist_[i];

        for(size_t j=1;j<track.featureTrack.size();++j)
        {
            for(size_t k=0;k<j;k++)
            {
                int v1_img_id=track.featureTrack[k].view_id_;
                int v2_img_id=track.featureTrack[j].view_id_;

                int v1_feature_id=track.featureTrack[k].feature_id_;
                int v2_feature_id=track.featureTrack[j].feature_id_;


                if(v1_img_id>v2_img_id)
                {
                    std::swap(v1_img_id,v2_img_id);
                    std::swap(v1_feature_id,v2_feature_id);
                }       
                
                //判断该图像对是否已经存在（不同的图像对，该值存在唯一值）
                int lookup=v1_img_id*num_views+v2_img_id;
                int pair_exist=candidate_lookup[lookup];

                //不存在的话，则添加到候选匹配对里去
                if(pair_exist==-1)
                {   
                    pair_exist=static_cast<int>(CandidatePairs_.size());
                    candidate_lookup[lookup]=pair_exist;
                    CandidatePairs_.emplace_back(CandidatePair());
                    CandidatePairs_.back().view_1_id=v1_img_id;
                    CandidatePairs_.back().view_2_id=v2_img_id;    
                }


                View const& view1=views[v1_img_id];
                View const& view2=views[v2_img_id];

                cv::Point2f p1=view1.keyPoitsTrack[v1_feature_id].pt;
                cv::Point2f p2=view2.keyPoitsTrack[v2_feature_id].pt;

                //将这对图像对对应的这条track的特征点，存入到match里，做为最后排序的准则
                CandidatePairs_[pair_exist].matches.emplace_back(point2d2d(v1_feature_id,v2_feature_id,p1,p2));

            }
        }
    }
    //按照匹配的数量，对图像对进行降序排序
    std::sort(CandidatePairs_.begin(),CandidatePairs_.end(),[](CandidatePair &l,CandidatePair &r){
        return l.matches.size()>r.matches.size();            
    });

#ifdef SORT_TEST
    printf("%s\n","CandidatePairs'matches size is:");
    for(size_t i=0;i<CandidatePairs_.size();i++)
    {
        printf("%d %s",CandidatePairs_[i].matches.size()," ");
    }
#endif

}


void
Increment_SFM::Initialize()
{
    assert(CandidatePairs_.size()>0);
    
    for(size_t i=0;i<CandidatePairs_.size();++i)
    {
        Return_Info_Init return_info_=Init_->RecoverPoseFromE(CandidatePairs_[i].matches,K_);
        if(return_info_.is_succeed==true)   //初始对重建成功
        {
            //将重建的初始对的图像id以及该图像的位姿添加到map里
            Maps_->AddImagePose(CandidatePairs_[i].view_1_id,return_info_.R1,return_info_.t1);
            Maps_->AddImagePose(CandidatePairs_[i].view_2_id,return_info_.R2,return_info_.t2);

            for(size_t k=0;k<return_info_.inlier_mask.size();++k)
            {
                //判断特征点匹配对是否有效
                if(!return_info_.inlier_mask[k])
                    continue;
                track_ temp_track;
                temp_track.push_back(trackElement(CandidatePairs_[i].view_1_id,CandidatePairs_[i].matches[k].feature_id1));
                temp_track.push_back(trackElement(CandidatePairs_[i].view_2_id,CandidatePairs_[i].matches[k].feature_id2));


                assert(views[CandidatePairs_[i].view_1_id].keyPoitsTrack[CandidatePairs_[i].matches[k].feature_id1].class_id==
                views[CandidatePairs_[i].view_2_id].keyPoitsTrack[CandidatePairs_[i].matches[k].feature_id2].class_id);

                cv::Vec3b temp_color=tracklist_[views[CandidatePairs_[i].view_1_id].keyPoitsTrack[CandidatePairs_[i].matches[k].feature_id1].class_id].color;
                
                //将该track注册
                tracklist_[views[CandidatePairs_[i].view_1_id].keyPoitsTrack[CandidatePairs_[i].matches[k].feature_id1].class_id].is_register=true;
                tracklist_[views[CandidatePairs_[i].view_1_id].keyPoitsTrack[CandidatePairs_[i].matches[k].feature_id1].class_id].pos=return_info_.points3D[k];
                tracklist_[views[CandidatePairs_[i].view_1_id].keyPoitsTrack[CandidatePairs_[i].matches[k].feature_id1].class_id].p3d_idx=Maps_->get_point3d_idx();

                //将三维点注册到map里去
                Maps_->AddPoint(return_info_.points3D[k],temp_track,temp_color,return_info_.residuals[i]);
            }
            break;
        }
    }
}


/*查找下一个重建的图像,主要思想选择的下一个图像，可以看到最多的已经重建的三维点*/
void 
Increment_SFM::find_next_img()
{
    std::vector<std::pair<int,int>> next_views(views.size());

    for(size_t i=0;i<next_views.size();++i)
    {
        next_views[i]=std::make_pair<int,int>(0,static_cast<int>(i));
    }

    for(size_t i=0;i<tracklist_.size();++i)
    {
        if(tracklist_[i].is_register==true)
        {
            for(size_t k=0;k<tracklist_[i].featureTrack.size();++k)
            {
                if(std::count(Maps_->get_regist_img().begin(),Maps_->get_regist_img().end(),tracklist_[i].featureTrack[k].view_id_)>0)
                    continue;
                next_views[tracklist_[i].featureTrack[k].view_id_].first+=1;
            }
        }
    }
    sort(next_views.begin(),next_views.end(),[](std::pair<int,int>&l,std::pair<int,int>&r){
        return l.first>=r.first;
    });

    //获取下一个要重建的图像，按照可以看到的三维点的数量来排序
    for(auto &t:next_views)
    {
        next_imgs.push_back(t.second);
    }
}

/*获取2d-3d的对应关系，为了PNP计算相机的位姿做准备*/
void 
Increment_SFM::Get2d3d_match(const int&img_id,
                            std::vector<cv::Vec2f> &point2ds,
                            std::vector<cv::Vec3f> &point3ds,
                            std::vector<int>       &point2d_idxs,
                            std::vector<int>       &point3d_idxs 
                            )
{
    point2ds.clear();
    point3ds.clear();
    point2d_idxs.clear();
    point3d_idxs.clear();

    const View &img_=views[img_id];

    for(size_t point2d_idx=0;point2d_idx<img_.keyPoitsTrack.size();++point2d_idx)
    {
        if(tracklist_[img_.keyPoitsTrack[point2d_idx].class_id].is_register==true)
        {
            point2ds.push_back(img_.keyPoitsTrack[point2d_idx].pt);
            point3ds.push_back(tracklist_[img_.keyPoitsTrack[point2d_idx].class_id].pos);
            point2d_idxs.push_back(point2d_idx);
            point3d_idxs.push_back(tracklist_[img_.keyPoitsTrack[point2d_idx].class_id].p3d_idx);
        }
    }  
}
/*重建新的track，也就是新的三维点*/
void
Increment_SFM::Triangulate_new_tracks()
{   
    for(size_t i=0;i<tracklist_.size();++i)
    {
        std::vector<cv::Point2f> point_pos;
        //std::vector<registered_images_> cam_pose;
        std::vector<cv::Mat> Rs;
        std::vector<cv::Mat> ts;

        std::vector<int> img_ids;
        std::vector<int> feature_ids;

        if(tracklist_[i].is_register==true)
            continue;
        Track &no_register_track=tracklist_[i];
        for(size_t k=0;k<no_register_track.featureTrack.size();++k)
        {
            if(std::count(Maps_->get_regist_img().begin(),Maps_->get_regist_img().end(),
                          no_register_track.featureTrack[k].view_id_)==0)
            {
                continue;
            }
            point_pos.emplace_back(views[no_register_track.featureTrack[k].view_id_].keyPoitsTrack[
                                    no_register_track.featureTrack[k].feature_id_].pt);
            //cam_pose.emplace_back(Maps_->get_regist_img_pose(no_register_track.featureTrack[k].view_id_));
            Rs.emplace_back(Maps_->get_regist_img_pose(no_register_track.featureTrack[k].view_id_).R_);
            ts.emplace_back(Maps_->get_regist_img_pose(no_register_track.featureTrack[k].view_id_).t_);

            img_ids.emplace_back(no_register_track.featureTrack[k].view_id_);
            feature_ids.emplace_back(no_register_track.featureTrack[k].feature_id_);
        }
        //TODO:三角测量
        if(point_pos.size()<2)
            continue;
        Return_Info_Tri return_info_;
        return_info_=Init_->Triangulate(Rs,ts,Utils::point2fToVec2ds(point_pos),K_);//多个匹配点对应的三角量测

        track_ temp_track;
        for(size_t i=0;i<return_info_.inliers_.size();++i)
        {
            if(!return_info_.inliers_[i])
                continue;
            temp_track.push_back(trackElement(img_ids[i],feature_ids[i])); //将内点，存在到track（地图）里去
        }
        
        cv::Vec3b temp_color=no_register_track.color;

        //将该track注册
        no_register_track.is_register=true;
        no_register_track.pos=return_info_.point3D;
        no_register_track.p3d_idx=Maps_->get_point3d_idx();

        Maps_->AddPoint(return_info_.point3D,temp_track,temp_color,return_info_.ave_residual);
    }

}


void
Increment_SFM::Sfm()
{   
    //初始对的选择与重建
    Initialize();
    while(true)
    {
        //找到下一个重建的图像的候选图像集
        find_next_img();
        //所有的图像都重建完成
        if(next_imgs.size()==0)
        {
            break;
        }

        for(const int& img_id:next_imgs)
        {   
            std::vector<cv::Vec2f> point2ds;
            std::vector<cv::Vec3f> point3ds;
            std::vector<int>       point2d_idxs;
            std::vector<int>       point3d_idxs; 
            Get2d3d_match(img_id,point2ds,point3ds,point2d_idxs,point3d_idxs);

            Return_Info_P3d2d return_info_;
            return_info_=PnpSlover_->PnP3d2dSolve(Utils::vec3fToPoint3f(point3ds),Utils::vec2fToPoint2f(point2ds),K_);

            if(return_info_.is_success==true)
            {
                Maps_->AddImagePose(img_id,return_info_.R,return_info_.t);

                for(size_t i=0;i<return_info_.inlier_mask.size();++i)
                {
                    if(!return_info_.inlier_mask[i])
                        continue;
                    const int& point2D_idx = point2d_idxs[i];
                    const int& point3D_idx = point3d_idxs[i];

                    trackElement track_single(img_id,point2D_idx);
                    const double &error=return_info_.residuals[i];

                    Maps_->AddExistPoint(point3D_idx,track_single,error);
                }
                //TODO:开始重建新的三维点
                Triangulate_new_tracks();
                break;
            }   
        }
        next_imgs.clear();
    }

}


}