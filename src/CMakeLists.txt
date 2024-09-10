add_library(myslam SHARED
        backend.cpp
        camera.cpp
        config.cpp
        dataset.cpp
        feature.cpp
        frame.cpp
        frontend.cpp
        map.cpp
        mappoint.cpp
        viewer.cpp
        visual_odometry.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})