#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/frame.h"
// frame.h里面有camera.h实际上已经包含了


namespace myslam {

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    bool AddFrame(Frame::Ptr frame);

    void SetMap(Map::Ptr map) { map_ = map;}

    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer;}

    FrontendStatus GetStatus() const { return status_; }

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    // From ORB_SLAM3
    std::list<SE3> mlRelativeFramePoses;
    std::list<std::weak_ptr<Frame>> mlpReferences;
    // std::list<double> mlFrameTimes;
    // std::list<bool> mlbLost;
  
private:

    bool Track();

    bool Reset();

    // @return num of tracked points
    int TrackLastFrame();

    // @return num of inliners
    int EstimateCurrentPose();

    bool InsertKeyframe();

    bool StereoInit();

    int DetectFeatures();

    int FindFeaturesInRight();

    bool BuildInitMap();

    int TriangulateNewPoints();

    void SetObservationsForKeyFrame();
    

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    // 注意当前帧和上一帧始终有share_ptr,如果再之前的帧不是关键帧就会被销毁
    // 如果是关键帧，map_有一个unorder_map会一直存放这一帧的Ptr，
    // 当前关键帧也会有一个Ptr一直指向
    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Frame::Ptr mpCurrentKF = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;

    int tracking_inliners_ = 0; // used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;

    
};

} // namespace mysalm






#endif