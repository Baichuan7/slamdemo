
#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "myslam/g2o_types.h"

namespace myslam{

Backend::Backend() {
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

//这个函数实际上是用来notify启动其他线程实现updatemap的
void Backend::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop() {
    backend_running_.store(false);
    // 源代码没有这一句，但是如果已经进了BackendLoop循环，但是还没有wait的时候notify了，那么BaokendLoop就会一直阻塞了
    std::unique_lock<std::mutex> lock(data_mutex_);//为啥没有这句
    map_update_.notify_one();//猜测：让阻塞的BackendLoop跑完
}

void Backend::BackendLoop() {
    while (backend_running_.load())
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        //仅优化active的Frames和lndmarks
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void Backend::Optimize(Map::KeyframesType& keyframes,
                       Map::LandmarksType& landmarks )
{
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
                           LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>())
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //Pose vertex
    std::map<unsigned long, VertexPose*> vertices;
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose* vertex_pose = new VertexPose();
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (max_kf_id < kf->keyframe_id_) {
            max_kf_id = kf->keyframe_id_;
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});
    }

    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // Landmarks vertex and Edges
    // 这是一个只连接active位姿和landmark的图，没有pose之间的边
    std::map<unsigned long, VertexXYZ*> vertices_landmarks;

    int index = 1;
    double chi2_th = 5.991;
    std::map<EdgeProjection*, Feature::Ptr> edges_and_features;//vertex和id，edge和feature

    // 对每一个landmark
    // 需要取出其对应的frame和feat
    // edge连对应frame的vertex 和 landmark
    // feat作为measurement
    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;//和landmark.first一致

        // 在这里加上landmark顶点的添加，本来放在obs循环里
        VertexXYZ* v = new VertexXYZ();
        v->setId(landmark_id + max_kf_id + 1);
        v->setEstimate(landmark.second->Pos());
        v->setMarginalized(true);
        vertices_landmarks.insert({landmark_id, v});
        optimizer.addVertex(v);

        auto observations = landmark.second->GetObs();
        for (auto& obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            // 回看对应的类的判断条件，如bool outlier或者weak_ptr
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
            auto frame = feat->frame_.lock();

            EdgeProjection* edge = nullptr;
            if (feat->is_on_left_image_) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // 加入landmark顶点
            // 但是这里放入了obs的循环里，给了一个vertices_landmarks重复放入的if判断
            // 我打算把他放到obs循环外，并且省区这个判断
            // if (vertices_landmarks.find(landmark_id) != vertices_landmarks.end())
            // {
            //     VertexXYZ* v = new VertexXYZ();
            //     v->setId(landmark_id + max_kf_id + 1);
            //     v->setEstimate(landmark.second->Pos());
            //     v->setMarginalized(true);
            //     vertices_landmarks.insert({landmark_id, v});
            //     optimizer.addVertex(v);
            // }

            // 先判断edge要连的两个顶点都存在
            if (vertices.find(frame->keyframe_id_) != vertices.end()
             && vertices_landmarks.find(landmark_id) != vertices_landmarks.end())
            {
                edge->setId(index);index++;
                edge->setVertex(0, vertices.at(frame->keyframe_id_));
                edge->setVertex(1, vertices_landmarks.at(landmark_id));
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
            }
            else delete edge;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5)
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        for (auto& ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th)
            {
                cnt_outlier++;
            }
            else cnt_inlier++;
        }

        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) break;
        else
        {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto& ef : edges_and_features)
    {
        if (ef.first->chi2() > chi2_th)
        {
            ef.second->is_outlier_ = true;
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        }
        else 
        {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "OUTLIER/INLIER in Optimization: " << cnt_outlier << "/" << cnt_inlier;

    for (auto& v : vertices)
    {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto& v : vertices_landmarks)
    {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }

}
}