cmake_minimum_required(VERSION 2.8)
project(myslam) # ${PROJECT_NAME}

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall") # 传递给C++编译器的编译标志，c++17标准，启用编译器所有常见警告
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -O3 -fopenmp -pthread")# 在release下的编译标志 -03最高级别优化 -fopenmp启用OpenMP支持（多线程并行编程API) -pthread启用POSIX线程支持

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)# list append 用于在列表变量追加元素，CMAKE_MODULE_PATH是查找.cmake模块文件的列表
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# PROJECT_SOURCE_DIR 是包含 CMakeLists.txt 文件的目录。
# PROJECT_BINARY_DIR 是运行 cmake 命令时指定的构建目录。

######################dependencies###########################
#Eigen
include_directories("/usr/include/eigen3")# 添加头文件搜索路径

#OpenCV
find_package(OpenCV 4.4 REQUIRED)# 设置了OPENCV_INCLUDE_DIRS（头文件目录），OpenCV_LIBS（需要链接的库），OpenCV_VERSION（版本），OpenCV_FOUND（bool）
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# GFlags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES} 
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        # g2o 似乎find_package中不会定义链接库的变量
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY})

enable_testing()# 启用项目的测试功能，可以使用add_test()定义具体的测试


############### source and test #############
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)# 子目录 方便维护独立的模块
add_subdirectory(app)
add_subdirectory(test)