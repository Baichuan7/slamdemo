#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRRONTEND_H

#include <opencv2/features2d.hpp>
#include "myslam/map.h"
#include "myslam/frame.h"


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
    
private:

    bool Track();

    bool Reset();

    // @return num of tracked points
    int TrackLastFrame();

    // @return num of inliners
    int EstiamteCurrentPose();

    bool InsertKeyframe();

    bool StereoInit();

    int DetectFeatures();

    int FindFeaturesInRight();

    bool BuildInitMap();

    int TriangulateNewPoints();

    void SetObservationsForKeyFrame();
    

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame = nullptr;
    Frame::Ptr last_frame = nullptr;
    Camera::Ptr camera_left = nullptr;
    Camera::Ptr camera_right = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;

    int tracking_inliners_ = 0; // used for testing new keyframes

    // params
    int num_features = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;

    
};

} // namespace mysalm






#endif