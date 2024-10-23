
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam {

VisualOdometry::VisualOdometry(const std::string& config_file) : config_file_path_(config_file) {}

bool VisualOdometry::Init() {
    // read from config file 
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }
    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);//glog

    //create components and links
    frontend_ = Frontend::Ptr(new Frontend());
    backend_ = Backend::Ptr(new Backend());
    map_ = Map::Ptr(new Map());
    viewer_ = Viewer::Ptr(new Viewer());
    
    frontend_->SetMap(map_);
    frontend_->SetBackend(backend_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running.";
        if (Step() == false) 
            break;
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit.";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr ) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    vTimesTrack.push_back(time_used.count());
    return success;
}

// Pose of KF is in map_
// relative pose from every frame to KF is in frontend
void VisualOdometry::SaveTrajectoryKITTI(const std::string& filename) {
    LOG(INFO) << "Saving camera trajectory to" << filename << " ...";

    // 去第一帧位姿，后面把轨迹的起点置为原点
    // 一般loop closure之后起点可能不是原点了 我这里暂时不需要
    SE3 Tow = map_->GetAllKeyFrames().at(0)->Pose().inverse();
    
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;
    // frontend_->mlpReferences[i].lock()

    std::list<std::weak_ptr<Frame>>::iterator lRit = frontend_->mlpReferences.begin();
    for(std::list<SE3>::iterator lit = frontend_->mlRelativeFramePoses.begin(),
    lend = frontend_->mlRelativeFramePoses.end();  lit!=lend; lRit++, lit++) {
        auto pKF = (*lRit).lock();
        if (!pKF) LOG(ERROR) << "This KF is nullptr.";
        SE3 Trw = pKF->Pose() * Tow;
        SE3 Tcw = (*lit) * Trw;
        SE3 Twc = Tcw.inverse();
        Eigen::Matrix3d Rwc = Twc.rotationMatrix();
        Eigen::Vector3d twc = Twc.translation();

        f << std::setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << std::endl;
    }
    f.close();
}

}