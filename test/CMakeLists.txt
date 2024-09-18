set(TEST_SOURCES test_triangulation)

foreach (test_src ${TEST_SOURCES})
    add_executable(${test_src} ${test_src}.cpp)
    target_link_libraries(${test_src} ${THIRD_PARTY_LIBS} myslam)
    add_test(${test_src} ${EXECUTABLE_OUTPUT_PATH}/${test_src}) # 测试名 要运行的命令（可执行文件或脚本）
endforeach (test_src)