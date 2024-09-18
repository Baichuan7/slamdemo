#ifndef MYSLAM_DATASET_H 
#define MYSLAM_DATASET_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include "myslam/frame.h"

namespace myslam {

/*
传入配置文件中的数据集路径
初始化相机
读取下一帧图像*/
class Dataset {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& dataset_path);

    bool Init();

    Frame::Ptr NextFrame();

    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
}
#endif





