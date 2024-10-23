#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

Frontend::Frontend() {
    gftt_ = 
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    // 这里好像是用上一帧和上上一帧的相对位姿乘以上一帧的位姿: 这里只是给个初值，在EstimateCurrentPose()会优化（待确定）
    if (last_frame_) {
        // 这里没有用几何方法计算相对位置 都是上一帧和上两帧的相对位姿
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliners_ = EstimateCurrentPose();

    if (tracking_inliners_ > num_features_tracking_)
    {
        status_ = FrontendStatus::TRACKING_GOOD;
    } 
    else if (tracking_inliners_ > num_features_tracking_bad_)
    {
        status_ = FrontendStatus::TRACKING_BAD;
    }
    else
    {
        status_ = FrontendStatus::LOST;
    }

    // InsertKeyframe(); 
    if(InsertKeyframe()) { mpCurrentKF = current_frame_; }
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    // Idea From ORBSLAM3
    // 为了储存所有轨迹
    // 为所有帧建立参考的关键帧，并且储存时间辍和相对位姿
    // 如果后端优化所有激活的KF了，那么Tcr_就不准了，但是要求实时出位姿时，记录此刻位姿是没问题的
    current_frame_->mpReferenceKF = mpCurrentKF;
    if(auto ref_kf = current_frame_->mpReferenceKF.lock()) {
        SE3 Tcr_ = current_frame_->Pose() * ref_kf->Pose().inverse();
        mlRelativeFramePoses.push_back(Tcr_);
        mlpReferences.push_back(current_frame_->mpReferenceKF);
    }
    else {
        std::cerr << "Fail to lock mpReferenceKF." << std::endl;
    }
    // mlFrameTimes.push_back(current_frame_->time_stamp_);

   if(viewer_) viewer_->AddCurrentFrame(current_frame_);
   return true;
}

/*
/ 观测的inlier不够就把当前帧设为关键帧
/ 地图管理关键帧
/ 和地图点
/ */
bool Frontend::InsertKeyframe() {
    if (tracking_inliners_ >= num_features_needed_for_keyframe_) {
        // 这里的插入策略采取跟踪的特征点不够的时候才补关键帧
        return false;
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe " 
            << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();
    // 观测不够插入关键帧，然后在这里补观测
    FindFeaturesInRight();
    TriangulateNewPoints();
    backend_->UpdateMap();

    if ( viewer_ ) viewer_->UpdateMap();

    return true;
}

/*
/ TrackLastFrame()中 新feature里的mappoint对应上了地图点
/ 这里把mappoint中的observations对应上这里的新feature
*/
void Frontend::SetObservationsForKeyFrame() {
    for (auto& feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

/*
仅仅从同一帧左右目恢复3D点
*/
int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    size_t cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); i++) {
        // 左图有没有对应地图点且有关联的右图点
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {

            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                            current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                            current_frame_->features_right_[i]->position_.pt.y))
            };
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_triangulated_pts++;
                map_->InsertMapPoint(new_map_point);
            }
        }
    }
    LOG(INFO) << "new landmark " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();

    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

/*
/ 因为特征点不够，所以这一帧设为关键帧
/ 然后提取更多特征
*/
int Frontend::DetectFeatures() {
    // 先做一个mask，在已有特征点的10X10邻域内填上0(mask中非0值是ROI），这样防止重复检测过近的特征点
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto& feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                        feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }
    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for( auto& kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp))//相当于构造一个匿名的shared_ptr类对象push进features_left_里面
        );
        cnt_detected++;
    }

    LOG(INFO) << "Detected " << cnt_detected << " new features";
    return cnt_detected;
}

/*
光流跟踪左右目图像
*/
int Frontend::FindFeaturesInRight() {
    std::vector<cv::Point2f> kps_left, kps_right;
    // 和TrackLastFrame()一致，
    for(auto& kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        }
        else {
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3, 
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        }
        else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " features in the right image.";
    return num_good_pts;
}

// 跟踪这一帧的所有特征 
// 并且把新的特征对应上地图点
int Frontend::TrackLastFrame() {
    std::vector<cv::Point2f> kps_last, kps_current;
    /*
    // 检查上一帧左图所有特征的是否指向地图点
    // 如果没有 则把上一帧的特征位置放到kps_last和kps_current
    // 如果有 则把地图点重投影到这一帧图像上，放入kps_current, 上一帧特征位置仍放入kps_last
    */
    for (auto& kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            auto mp = kp->map_point_.lock();
            // 这里知道了所有每一feature里指向的地图点都是在世界系下的
            // 这里恢复的像素坐标作为LK估计的初值
            auto px = 
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        }
        else{
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    //还是在前后帧左图之间光流追踪，右图的点似乎只是用作初值
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current, 
        status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW
    );

    int num_good_pts = 0;

    // 在当前帧中添加特征，光流跟踪上的特征都加入current_frame_中
    // 每个特征都对应上地图点
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);//7是领域大小，用来给描述子指定邻域大小
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

/*
/ 图优化pose
/ 用当前帧pose和观测构建单节点图
/ chi2大的观测设为outlier，reset对应mappoint
/ @return 观测内点数量
*/
int Frontend::EstimateCurrentPose () {
    // 建立图模型，vertex是这一帧的pose，边是这一帧的所有观测
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    //PoseMatricType就是维度为6（上面6是pose dim，3是landmark dim，PoseMatrixType即6*6，即Ax=b的A，这里A是一个对称的半正定矩阵）的eigen matrix
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()
        )
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    VertexPose* vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    Mat33 K = camera_left_->K();

    int index = 1;
    std::vector<EdgeProjectionPoseOnly*> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); i++)
    {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if(mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly* edge = new EdgeProjectionPoseOnly(mp->Pos(), K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    //筛选外点
    // 10次优化后，计算当前当前外点的误差，然后对所有点，检测其卡方（chi2=error*\omega*error)大小普安段是否为外点
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;
        // 外点的边会setevel(1), 可以在optimizer.initializeOptimization(int )中选择优化的范围，不会优化>=int的边
        for(size_t i = 0; i < edges.size(); i++) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {e->computeError();}
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            }
            else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }
            //鲁棒核防止误差过大，但是会影响优化精度，默认最后一次优化前外点筛完了，进行一次不加核函数的优化
            if(iteration == 2) {e->setRobustKernel(nullptr);}
        }
    }

    LOG(INFO) << "Outlier/Inlier in Pose estimation: " << cnt_outlier << "/" <<
                features.size() - cnt_outlier;

    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current pose: \n" << current_frame_->Pose().matrix();

    // 这里是不是应该直接对 current_frame->features_left_操作 
    // features只是拷贝过来的
    // 但是上面outlier点的判断也是在features里设置的
    // 没事了 里面放的是指针 实际操作的还是current_frame
    for (auto& feat : features)
    {
        if (feat->is_outlier_) {
            feat->map_point_.reset();//注意这是shared_ptr的方法
            feat->is_outlier_ = false;
        }
    }
        //返回内点数量
        return features.size() - cnt_outlier;
    }

// 用左右帧进行三角化后建图
bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); i++) {
        if (current_frame_->features_right_[i] == nullptr) continue;

        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                        current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                        current_frame_->features_right_[i]->position_.pt.y))
        };
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    // 这里只在初始化调用一次，所以第一帧一定为关键帧并且是世界系，所有没有对pworld转系，也临时直接在这里set了keyframe
    // frame->pose没有显式初始化，默认为单位阵
    current_frame_->SetKeyFrame();//难道不是先设定为keyframe再建立地图吗，放到建图函数里面干什么
    mpCurrentKF = current_frame_;
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    current_frame_->mpReferenceKF = mpCurrentKF;
    SE3 Tcr_ = SE3();
    mlRelativeFramePoses.push_back(Tcr_);
    mlpReferences.push_back(current_frame_->mpReferenceKF);
    // mlFrameTimes.push_back(current_frame_->time_stamp_);


    LOG(INFO) << "Initial map created with " << cnt_init_landmarks << " map points.";
    return true;
}

bool Frontend::Reset() {
    LOG(INFO) << "REset is not implemented.";
    return true;
}

}