# TRAC-IK Standalone (Non-ROS Version)

[![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## 📖 概述

这是 **TRAC-IK** 的独立编译版本，移除了对 ROS 的依赖，可以在**非 ROS 环境**下直接使用。

TRAC-IK 是一个高性能的逆运动学（IK）求解器，专门设计用于处理关节限位约束。相比传统的 KDL IK 求解器，TRAC-IK 在存在关节限位的情况下具有更高的求解成功率和更快的速度。

## ✨ 主要特性

- ✅ **支持关节限位** - 自动考虑关节上下限约束
- ✅ **高求解成功率** - 相比 KDL 提升显著（99%+）
- ✅ **快速求解** - 平均求解时间 < 1ms
- ✅ **非 ROS 环境** - 无需 ROS，可直接集成到任何 C++ 项目
- ✅ **向后兼容** - 支持条件编译，可选择 ROS 或非 ROS 版本

## 🔧 主要修改

本版本相比原始 TRAC-IK 的主要改动：

- ✅ 移除了 ROS (rclcpp) 硬依赖
- ✅ 添加了 `rclcpp_logger_stub.hpp` 作为日志替代实现
- ✅ 支持条件编译（通过 `USE_ROS` 宏）
- ✅ 添加了独立的 CMakeLists.txt
- ✅ **保留了所有核心 IK 功能**

## 📋 依赖项

### 必需依赖

- **KDL (Orocos Kinematics and Dynamics Library)** - 机器人运动学库
- **Eigen3** - 线性代数库
- **NLopt** - 非线性优化库

### 安装依赖

#### Windows (使用 vcpkg)
```powershell
vcpkg install nlopt:x64-windows
vcpkg install eigen3:x64-windows
```

#### Linux (Ubuntu/Debian)
```bash
sudo apt-get install libnlopt-dev libeigen3-dev
```

#### macOS (使用 Homebrew)
```bash
brew install nlopt eigen
```

## 🚀 快速开始

### 编译

```bash
cd trac_ik/trac_ik_lib
mkdir build
cd build

# Windows (使用 vcpkg)
cmake .. -DCMAKE_BUILD_TYPE=Release ^
         -DUSE_ROS=0 ^
         -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

# Linux/Mac
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ROS=0

# 编译
cmake --build . --config Release
```

### 使用示例

**快速示例**：

```cpp
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

// 1. 创建机器人链
KDL::Chain chain;
// ... 添加 segments（见 API_USAGE_EXAMPLES.md）...

// 2. 设置关节限位
KDL::JntArray q_min(6), q_max(6);
q_min(0) = -3.14; q_max(0) = 3.14;
// ... 设置其他关节限位 ...

// 3. 创建 TRAC-IK 求解器
TRAC_IK::TRAC_IK ik_solver(
    chain, q_min, q_max,
    0.005,        // timeout (秒)
    1e-5,         // error tolerance
    TRAC_IK::Speed
);

// 4. 求解 IK
KDL::Frame target_pose;
KDL::JntArray q_init(6), q_result(6);
int result = ik_solver.CartToJnt(q_init, target_pose, q_result);

if (result >= 0) {
    std::cout << "IK solved! Found " << result << " solutions." << std::endl;
}
```

**更多示例**：
- 📖 [完整 API 使用指南](API_USAGE_EXAMPLES.md) - 详细的使用示例和最佳实践
- 💻 [可运行示例程序](examples/basic_ik_example.cpp) - 完整的编译运行示例

## 🔧 集成到项目

### Visual Studio 项目设置

1. **添加包含目录**：
   ```
   ..\trac_ik\trac_ik_lib\include
   ```

2. **添加库目录**：
   ```
   ..\trac_ik\trac_ik_lib\build\Release
   C:\vcpkg\installed\x64-windows\lib
   ```

3. **添加库依赖**：
   ```
   trac_ik.lib
   nlopt.lib
   orocos-kdl.lib
   ```

### CMake 项目集成

```cmake
# 添加包含目录
include_directories(${CMAKE_SOURCE_DIR}/trac_ik/trac_ik_lib/include)

# 链接库目录
link_directories(${CMAKE_SOURCE_DIR}/trac_ik/trac_ik_lib/build/Release)

# 链接库
target_link_libraries(your_target
    trac_ik
    nlopt
    orocos-kdl
)
```

## ⚠️ 故障排除

### 找不到 KDL
```bash
# 设置 KDL 路径
cmake .. -DKDL_DIR=/path/to/kdl/install
```

### 找不到 NLopt
```bash
# Windows (vcpkg)
cmake .. -DCMAKE_PREFIX_PATH="C:/vcpkg/installed/x64-windows"

# Linux
sudo apt-get install libnlopt-dev
```

### 链接错误
确保链接了所有依赖：
- `trac_ik.lib` (或 `.a`)
- `orocos-kdl.lib`
- `nlopt.lib`
- `Eigen3` (通常是头文件库)

### 编译错误：找不到 rclcpp
确保编译时设置了 `USE_ROS=0`（默认值）。源码已修改完成，无需手动修改。

## 📊 性能对比

根据原始 TRAC-IK 的测试结果（10,000 次随机可达配置测试）：

| 机器人 | DOFs | KDL 成功率 | TRAC-IK 成功率 | TRAC-IK 平均时间 |
|--------|------|-----------|---------------|-----------------|
| ABB IRB120 | 6 | 39.41% | **99.96%** | 0.24ms |
| KUKA LBR iiwa 14 | 7 | 38.09% | **99.92%** | 0.28ms |
| Universal UR5 | 6 | 16.52% | **99.17%** | 0.37ms |
| Franka Emika Panda | 7 | 62.02% | **99.88%** | 0.37ms |

## 📚 文档

- [API 使用示例](API_USAGE_EXAMPLES.md) - 详细的 API 使用指南和代码示例
- [修改记录](CHANGELOG.md) - 版本更新历史

## ⚖️ License & Copyright

This project is a modified version of [TRAC-IK](https://bitbucket.org/traclabs/trac_ik).

**Original Copyright**: Copyright (c) 2015, TRACLabs, Inc.  
**License**: BSD 3-Clause License

This modified version maintains the same license and copyright requirements.

See [LICENSE.txt](LICENSE.txt) for the full license text.

## 🙏 致谢

感谢 **TRACLabs, Inc.** 开发了优秀的 TRAC-IK 库。

## 🔗 相关链接

- [原始 TRAC-IK 项目](https://bitbucket.org/traclabs/trac_ik)
- [TRAC-IK 论文](https://www.researchgate.net/publication/282852814_TRAC-IK_An_Open-Source_Library_for_Improved_Solving_of_Generic_Inverse_Kinematics)
- [KDL 项目](https://www.orocos.org/kdl.html)
- [NLopt 项目](https://nlopt.readthedocs.io/)

---

**注意**: 本版本专注于非 ROS 环境使用。如果需要 ROS 支持，请使用原始 TRAC-IK 项目或设置 `USE_ROS=1` 编译。
