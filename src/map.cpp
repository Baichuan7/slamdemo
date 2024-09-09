#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    if (landmarks_.find(map_point_->id_) == landmarks_.end())
    {
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id] = map_point;
        active_landmarks_[map_point->id] = map_point;
    }
}

void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end())
    {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframe_) {
        RemoveOldKeyframe();
    }
}

void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;// 这是干嘛
    // 计算当前帧和激活的关键帧内pose距离
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();
    for (auto& kf : active_keyframes_)
    {
        if (kf == current_frame_) continue;
        auto dis = (kf.second->Pose * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    // 如果距离过近，删除该帧，否则删除距离最远的
    const double min_dis_th = 0.2;
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        frame_to_remove = keyframes_.at(max_kf_id);
    }
    
    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;

    active_keyframes.erase(frame_to_remove->keyframe_id_);
    // 删除地图点在该帧的观测
    for (auto feat : frame_to_remove->features_right_) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) mp->RemoveObservation(feat);
    }
    for (auto feat : frame_to_remove->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->RemoveObservation(feat);
    }

    CleanMap();
}

void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    // 有删去迭代器的操作时把循环递增放在循环体内
    for (auto iter = active_landmarks_.begin(); iter!=active_landmarks_.end();)
    {
        if (iter->second->observed_times_ == 0) {
            iter = active_landmarks.erase(iter);// erase返回删除元素之后的迭代器
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }

    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks.";
}

}