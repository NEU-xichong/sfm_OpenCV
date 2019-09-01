#include"map.h"

namespace FeatureGraph{


Map::Map()
{
    num_point3d_idx=0;
}

void 
Map::AddPoint(const cv::Vec3d& point_,const track_ m_track_,const cv::Vec3b color,const double err)
{
    point3ds[num_point3d_idx].pos_=point_;
    point3ds[num_point3d_idx].tracks=m_track_;
    point3ds[num_point3d_idx].color_=color;
    point3ds[num_point3d_idx].err=err;
    point3ds[num_point3d_idx].idx_=num_point3d_idx;


    num_point3d_idx++;
}

void
Map::AddExistPoint(const point3d_idx& point3D_idx_, const trackElement& track_el, const double& error)
{
    assert(point3ds.find(point3D_idx_)!=point3ds.end());

    
    //计算平均误差
    point3ds[num_point3d_idx].err=(point3ds[num_point3d_idx].err*point3ds[point3D_idx_].tracks.size()+error)/
                                  (point3ds[point3D_idx_].tracks.size()+1);

    //将新的能观测到三维点的image_id和特征点id放到三维点对应的track里
    point3ds[point3D_idx_].tracks.push_back(track_el);
}

void 
Map::AddImagePose(const int image_id,const cv::Mat R,const cv::Mat t)
{
    images_.insert(std::pair<int,registered_images_>(image_id,registered_images_(R,t)));
    registered_.emplace_back(image_id);

}



registered_images_
Map::get_regist_img_pose(const int img_id){
    return images_[img_id];
}



point3d_idx
Map::get_point3d_idx()
{
    return num_point3d_idx;
}


std::vector<int>
Map::get_regist_img()
{
    return registered_;
}







}