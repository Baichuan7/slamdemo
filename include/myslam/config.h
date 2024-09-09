#pragma once 
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

// singleton
// 不return static的对象
// SetParameterFile建立对象的static指针 Get中用这个指针实现想要的功能 
class Config {
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config();
public:
    ~Config();

    //singleton：建立唯一对象的指针，初始化file_
    static bool SetParameterFile(const std::string& filename);

    template <typename T>
    static T Get(const std::string& key) {
        return T(Config::config_->file_[key]);
    }
};
}

#endif