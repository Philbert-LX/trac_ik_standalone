# TRAC-IK 示例程序 CMakeLists.txt

cmake_minimum_required(VERSION 3.5)
project(trac_ik_examples)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(Eigen3 REQUIRED)
find_package(orocos-kdl REQUIRED)

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../trac_ik_lib/include
    ${EIGEN3_INCLUDE_DIR}
    ${orocos-kdl_INCLUDE_DIRS}
)

# 链接库目录
link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../trac_ik_lib/build/Release
    ${CMAKE_CURRENT_SOURCE_DIR}/../trac_ik_lib/build
)

# 基本 IK 示例
add_executable(basic_ik_example basic_ik_example.cpp)

target_link_libraries(basic_ik_example
    trac_ik
    ${orocos-kdl_LIBRARIES}
)

# 编译选项
if(MSVC)
    target_compile_options(basic_ik_example PRIVATE /W4)
else()
    target_compile_options(basic_ik_example PRIVATE -Wall -Wextra)
endif()

