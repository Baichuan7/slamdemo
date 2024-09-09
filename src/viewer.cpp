
#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframe_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {
    // 创建并绑定窗口
    // 设置深度测试和混合函数 以便在渲染是处理透明度
    pangolin::CreateWindowAndBind("MySlam", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 定义投影矩阵和视图矩阵
    // 定义窗口尺寸 焦距 视口中心 和摄像机的位置和朝向
    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookat(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // 设置视口边界和比例，提供一个3D处理器Handler3D，用来通过鼠标键盘与场景交互V
    pangolin::View& vis_display = 
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));
    
    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};
    // 主循环
    while (!pangolin::ShouldQuit() && viewer_running_) {
        // 清除颜色和深度缓冲区 设置背景颜色为白色
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT_);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        //开始渲染当前帧
        vis_display.Activate(vis_camera);
        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame, green);
            FollowCurrentFrame(vis_camera);
            cv::Mat img = PlotFrameImage();
            cv::imshow("image", img);
            cv::WaitKey(1);
        }
        if (map_) {
            DrawMapPoints();
        }
        pangolin::FinishFrame();
        usleep(5000);
    }
    LOG(INFO) << "STOP VIEWER";
}

// 在OpenGL中绘制一个帧相机的坐标系和视锥，可视化位姿
void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
    SE3 Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;
    glPushMatrix();
    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());
    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);
    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glEnd();
    glPopMatrix();
}

// 使渲染摄像机跟随当前帧的方向和位置
void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, True);
}

// 在图像上可视化特征点
void Viewer::PlotFrameImage() {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(), ++i) {
        auto feat = current_frame_->features_left_[i];
        cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0), 2);
    }
    return img_out;
}

void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : acitive_keyframes_) {
        DrawFrame(kf, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    } 
    glEnd();
}

}