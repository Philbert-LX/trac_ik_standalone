# TRAC-IK Standalone CMakeLists.txt
# 用于在非 ROS 环境下编译 TRAC-IK
# 使用方法：将此文件重命名为 CMakeLists.txt，或使用 -f 选项

cmake_minimum_required(VERSION 3.5)
project(trac_ik_lib)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 定义 USE_ROS=0 以禁用 ROS 依赖
# 使用 option 允许用户在命令行覆盖
option(USE_ROS "Enable ROS support" OFF)
if(NOT USE_ROS)
    add_definitions(-DUSE_ROS=0)
    message(STATUS "Building TRAC-IK without ROS support (USE_ROS=0)")
else()
    add_definitions(-DUSE_ROS=1)
    message(STATUS "Building TRAC-IK with ROS support (USE_ROS=1)")
endif()

# 查找依赖
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

# 查找 KDL
find_package(orocos-kdl QUIET)
if(NOT orocos-kdl_FOUND)
    # 尝试使用 PkgConfig
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(KDL QUIET orocos-kdl)
        if(KDL_FOUND)
            set(orocos-kdl_FOUND TRUE)
            set(orocos-kdl_INCLUDE_DIRS ${KDL_INCLUDE_DIRS})
            set(orocos-kdl_LIBRARIES ${KDL_LIBRARIES})
        endif()
    endif()
endif()

# 如果还是找不到，尝试手动指定
if(NOT orocos-kdl_FOUND)
    message(WARNING "KDL not found via find_package, trying manual paths...")
    # 尝试常见路径
    set(KDL_POSSIBLE_PATHS
        "${CMAKE_SOURCE_DIR}/../../orocos_kinematics_dynamics-master/orocos_kdl"
        "${CMAKE_SOURCE_DIR}/../../../orocos_kinematics_dynamics-master/orocos_kdl"
        "C:/vcpkg/installed/x64-windows/include"
        "/usr/local/include"
        "/usr/include"
    )
    find_path(KDL_INCLUDE_DIR kdl/chain.hpp PATHS ${KDL_POSSIBLE_PATHS})
    if(KDL_INCLUDE_DIR)
        set(orocos-kdl_INCLUDE_DIRS ${KDL_INCLUDE_DIR})
        message(STATUS "Found KDL headers at: ${KDL_INCLUDE_DIR}")
    endif()
endif()

if(NOT orocos-kdl_FOUND AND NOT KDL_INCLUDE_DIR)
    message(FATAL_ERROR "KDL (Orocos Kinematics and Dynamics Library) not found. Please install it or set KDL_DIR.")
endif()

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
elseif(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIRS})
endif()

if(orocos-kdl_INCLUDE_DIRS)
    include_directories(${orocos-kdl_INCLUDE_DIRS})
elseif(KDL_INCLUDE_DIR)
    include_directories(${KDL_INCLUDE_DIR})
endif()

if(NLOPT_INCLUDE_DIRS)
    include_directories(${NLOPT_INCLUDE_DIRS})
endif()

# 源文件
set(TRAC_IK_SOURCES
    src/kdl_tl.cpp
    src/nlopt_ik.cpp
    src/trac_ik.cpp
)

# 创建库
add_library(trac_ik STATIC ${TRAC_IK_SOURCES})

# 链接库
target_link_libraries(trac_ik
    PRIVATE
)

# 添加 Eigen3
if(TARGET Eigen3::Eigen)
    target_link_libraries(trac_ik PRIVATE Eigen3::Eigen)
elseif(EIGEN3_LIBRARIES)
    target_link_libraries(trac_ik PRIVATE ${EIGEN3_LIBRARIES})
endif()

# 添加 KDL
if(TARGET orocos-kdl)
    target_link_libraries(trac_ik PRIVATE orocos-kdl)
elseif(orocos-kdl_LIBRARIES)
    target_link_libraries(trac_ik PRIVATE ${orocos-kdl_LIBRARIES})
elseif(KDL_LIBRARIES)
    target_link_libraries(trac_ik PRIVATE ${KDL_LIBRARIES})
else()
    # 尝试查找库文件
    find_library(KDL_LIB
        NAMES orocos-kdl kdl
        PATHS
            "${CMAKE_SOURCE_DIR}/../../orocos_kinematics_dynamics-master/orocos_kdl/build"
            "C:/vcpkg/installed/x64-windows/lib"
            "/usr/local/lib"
            "/usr/lib"
    )
    if(KDL_LIB)
        target_link_libraries(trac_ik PRIVATE ${KDL_LIB})
        message(STATUS "Found KDL library: ${KDL_LIB}")
    else()
        message(WARNING "KDL library not found, you may need to link it manually")
    endif()
endif()

# 添加 NLopt
if(TARGET NLopt::nlopt)
    target_link_libraries(trac_ik PRIVATE NLopt::nlopt)
elseif(NLOPT_LIBRARIES)
    target_link_libraries(trac_ik PRIVATE ${NLOPT_LIBRARIES})
else()
    find_library(NLOPT_LIB
        NAMES nlopt
        PATHS
            "C:/vcpkg/installed/x64-windows/lib"
            "/usr/local/lib"
            "/usr/lib"
    )
    if(NLOPT_LIB)
        target_link_libraries(trac_ik PRIVATE ${NLOPT_LIB})
        message(STATUS "Found NLopt library: ${NLOPT_LIB}")
    else()
        message(FATAL_ERROR "NLopt library not found. Please install NLopt.")
    endif()
endif()

# 设置包含目录（PUBLIC，供使用者包含）
target_include_directories(trac_ik
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 编译选项
if(MSVC)
    target_compile_options(trac_ik PRIVATE /W4 /wd4996)  # 禁用某些警告
    target_compile_definitions(trac_ik PRIVATE _USE_MATH_DEFINES)
else()
    target_compile_options(trac_ik PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# 确保 USE_ROS 定义传递给所有源文件
if(NOT USE_ROS)
    target_compile_definitions(trac_ik PRIVATE USE_ROS=0)
else()
    target_compile_definitions(trac_ik PRIVATE USE_ROS=1)
endif()

# 输出信息
message(STATUS "TRAC-IK Standalone Build Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Eigen3: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  KDL: ${orocos-kdl_INCLUDE_DIRS}")
message(STATUS "  NLopt: ${NLOPT_LIBRARIES}")

# 安装（可选）
install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS trac_ik
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

