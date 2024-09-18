#pragma once 
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/frontend.h"
#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/viewer.h"
#include "myslam/dataset.h"

namespace myslam {

class VisualOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    VisualOdometry(const std::string& config_file);

    bool Init();

    void Run();

    bool Step();

    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    Dataset::Ptr dataset_ = nullptr;
};
}

#endif