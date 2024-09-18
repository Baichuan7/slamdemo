#pragma once 

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam {

struct MapPoint;
struct Feature;
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;//为什么需要共享指针

    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    bool is_keyframe_ = false;
    double time_stamp_;
    SE3 pose_;
    std::mutex pose_mutex_;
    cv::Mat left_img_, right_img_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

public:
    Frame() {}

    Frame(long id, double time_stamp, const SE3& pose, const cv::Mat& left, const cv::Mat& right);

    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }


    void SetPose(const SE3& pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }
    
    void SetKeyFrame();
    //工厂设计模式，根据需要在这个函数里创建不同的Frame对象
    static std::shared_ptr<Frame> CreateFrame();
};

}

#endif