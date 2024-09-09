#pragma once 
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

struct Frame;

struct Feature;

struct MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;
    bool is_outlier_ = 0;
    Vec3 pos_ = Vec3::Zero();
    std::mutex data_mutex_; 
    int observed_times_ = 0;//being observed by feature matching algo
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    Vec3 Pos() {
        std::unique_mutex<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void setPos(const Vec& Pos) {
        std::unique_mutex<std::mutex> lck(data_mutex_);
        pos_ = Pos;
    }

    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_mutex<std::mutex> lck(data_mutex_);
        observations.push_back(feature);//这里的push是复制操作
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_mutex<std::mutex> lck(data_mutex);
        return observations_; 
    }
    //factory function
    static MapPoint::Ptr CreateNewMappoint();//也可以加MapPoint 不会报错

};

}

#endif