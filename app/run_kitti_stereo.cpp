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
    
    gflags::ShutDownCommandLineFlags();
    return 0;
}