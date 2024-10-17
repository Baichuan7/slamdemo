#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path"
);

int main (int argv, char** argc) {
    gflags::ParseCommandLineFlags(&argv, &argc, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file)
    );
    assert(vo->Init() == true);
    vo->Run();

    std::sort(vo->vTimesTrack.begin(), vo->vTimesTrack.end());
    const int nTimes = vo->vTimesTrack.size();
    LOG(INFO) << "median tracking time: " << vo->vTimesTrack[int(nTimes/2)];
    float totaltime = 0;
    for ( auto time : vo->vTimesTrack) {
        totaltime += time;
    }
    LOG(INFO) << "mean tracking time: " << totaltime/nTimes;
    gflags::ShutDownCommandLineFlags();
    return 0;
}