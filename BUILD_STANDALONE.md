# TRAC-IK 独立编译指南（非 ROS 环境）

## 概述

TRAC-IK 原本是为 ROS 设计的，但核心算法不依赖 ROS。本指南说明如何在非 ROS 环境下编译和使用 TRAC-IK。

## 依赖项

### 必需依赖
1. **KDL (Orocos Kinematics and Dynamics Library)**
   - 你已经有了 `orocos_kinematics_dynamics-master` 文件夹
   - 需要先编译 KDL

2. **Eigen3**
   - 线性代数库
   - 通常通过 vcpkg 或系统包管理器安装

3. **NLopt**
   - 非线性优化库
   - 需要单独安装

### 可选依赖（用于 URDF 解析）
- `urdf` - URDF 解析库（如果不需要从 URDF 加载，可以不用）
- `kdl_parser` - KDL 解析器（如果不需要从 URDF 加载，可以不用）

## 编译步骤

### 步骤 1：安装 NLopt

#### Windows (使用 vcpkg)
```powershell
vcpkg install nlopt:x64-windows
```

#### Linux
```bash
sudo apt-get install libnlopt-dev
```

### 步骤 2：编译 KDL（如果还没编译）

```bash
cd orocos_kinematics_dynamics-master/orocos_kdl
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
cmake --install . --prefix ../../../third_party/kdl_install
```

### 步骤 3：修改 TRAC-IK 源码（去除 ROS 依赖）

由于 TRAC-IK 使用了 `rclcpp::Logger`，我们需要创建一个 stub 来替换它。

#### 选项 A：使用提供的 stub（推荐）

1. 在 `trac_ik_lib/src/trac_ik.cpp` 开头添加：
```cpp
// 在文件最开头，在所有 include 之前
#ifndef USE_ROS
#define USE_ROS 0
#endif

#if !USE_ROS
#include <trac_ik/rclcpp_logger_stub.hpp>
#endif
```

2. 在 `trac_ik_lib/src/nlopt_ik.cpp` 开头添加：
```cpp
#ifndef USE_ROS
#define USE_ROS 0
#endif

#if !USE_ROS
#include <trac_ik/rclcpp_logger_stub.hpp>
#endif
```

3. 修改 `trac_ik_lib/include/trac_ik/trac_ik.hpp`：
```cpp
// 在文件开头添加
#ifndef USE_ROS
#define USE_ROS 0
#endif

#if !USE_ROS
#include <trac_ik/rclcpp_logger_stub.hpp>
#else
#include <rclcpp/rclcpp.hpp>
#endif
```

4. 修改 `trac_ik_lib/include/trac_ik/nlopt_ik.hpp`：
```cpp
#ifndef USE_ROS
#define USE_ROS 0
#endif

#if !USE_ROS
#include <trac_ik/rclcpp_logger_stub.hpp>
#else
#include <rclcpp/rclcpp.hpp>
#endif
```

#### 选项 B：使用 CMake 定义宏

在 CMakeLists.txt 中添加：
```cmake
add_definitions(-DUSE_ROS=0)
```

### 步骤 4：编译 TRAC-IK

#### 方法 1：使用独立的 CMakeLists.txt

```bash
cd trac_ik/trac_ik_lib
mkdir build
cd build

# 配置（Windows）
cmake .. -DCMAKE_BUILD_TYPE=Release ^
         -DEigen3_DIR=C:/path/to/eigen3 ^
         -DKDL_DIR=C:/path/to/kdl ^
         -DNLOPT_DIR=C:/path/to/nlopt ^
         -DCMAKE_PREFIX_PATH="C:/vcpkg/installed/x64-windows"

# 配置（Linux）
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DEigen3_DIR=/usr/lib/cmake/eigen3 \
         -DKDL_DIR=/usr/local/lib/cmake/orocos_kdl \
         -DNLOPT_DIR=/usr/local

# 编译
cmake --build . --config Release

# 安装（可选）
cmake --install . --prefix ../../../third_party/trac_ik_install
```

#### 方法 2：使用 Visual Studio 项目

1. 使用 CMake GUI 生成 Visual Studio 项目
2. 打开生成的解决方案
3. 编译 `trac_ik` 项目

### 步骤 5：集成到你的项目

#### 在 X-Robot.vcxproj 中添加

1. **添加包含目录**：
```xml
<AdditionalIncludeDirectories>
  ..\trac_ik\trac_ik_lib\include;%(AdditionalIncludeDirectories)
</AdditionalIncludeDirectories>
```

2. **添加库目录**：
```xml
<AdditionalLibraryDirectories>
  ..\trac_ik\trac_ik_lib\build\Release;%(AdditionalLibraryDirectories)
</AdditionalLibraryDirectories>
```

3. **添加库依赖**：
```xml
<AdditionalDependencies>
  trac_ik.lib;nlopt.lib;%(AdditionalDependencies)
</AdditionalDependencies>
```

## 使用示例

```cpp
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

// 创建 TRAC-IK 求解器
KDL::Chain chain;  // 你的机器人链
KDL::JntArray q_min(6);  // 关节下限
KDL::JntArray q_max(6);  // 关节上限

// 设置限位值
q_min(0) = -3.14; q_max(0) = 3.14;
// ... 设置其他关节限位

// 创建求解器（使用非 ROS 构造函数）
TRAC_IK::TRAC_IK ik_solver(
    chain,
    q_min,
    q_max,
    0.005,  // timeout
    1e-5,   // error
    TRAC_IK::Speed
);

// 使用求解器
KDL::Frame target_pose;
KDL::JntArray q_init(6);
KDL::JntArray q_result(6);

int result = ik_solver.CartToJnt(q_init, target_pose, q_result);
if (result >= 0) {
    // 求解成功，q_result 包含结果
}
```

## 注意事项

1. **日志输出**：stub 版本的日志会输出到 `stderr`，可以通过重定向控制输出

2. **URDF 构造函数**：如果使用从 URDF 加载的构造函数，需要安装 `urdf` 和 `kdl_parser`。否则只使用 `Chain` 构造函数即可。

3. **线程安全**：TRAC-IK 基于 KDL，KDL 不是线程安全的。不要在同一个进程中创建多个 TRAC-IK 实例。

4. **编译选项**：确保使用 C++17 或更高版本。

## 故障排除

### 找不到 KDL
```bash
# 设置 KDL 路径
export KDL_DIR=/path/to/kdl/install
# 或在 CMake 中指定
-DKDL_DIR=/path/to/kdl/install
```

### 找不到 NLopt
```bash
# Windows (vcpkg)
-DCMAKE_PREFIX_PATH="C:/vcpkg/installed/x64-windows"

# Linux
sudo apt-get install libnlopt-dev
```

### 链接错误
确保链接了所有依赖：
- `trac_ik.lib` (或 `.a`)
- `orocos-kdl.lib`
- `nlopt.lib`
- `Eigen3` (通常是头文件库)

## 简化版本（如果不需要 URDF 支持）

如果你只需要核心 IK 功能，可以：
1. 移除 `trac_ik.cpp` 中从 URDF 加载的构造函数
2. 只保留使用 `KDL::Chain` 的构造函数
3. 这样可以完全移除 `urdf` 和 `kdl_parser` 依赖

