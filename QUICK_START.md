# TRAC-IK 快速开始指南（非 ROS 环境）

## 快速步骤

### 1. 安装依赖

#### Windows (使用 vcpkg)
```powershell
vcpkg install nlopt:x64-windows
vcpkg install eigen3:x64-windows
```

#### Linux
```bash
sudo apt-get install libnlopt-dev libeigen3-dev
```

### 2. 修改源码（必需）

按照 `MODIFY_SOURCE_FILES.md` 中的说明修改以下文件：
- `trac_ik_lib/include/trac_ik/trac_ik.hpp`
- `trac_ik_lib/include/trac_ik/nlopt_ik.hpp`
- `trac_ik_lib/src/trac_ik.cpp`

**关键修改**：将 `#include <rclcpp/rclcpp.hpp>` 替换为条件编译，使用我们提供的 `rclcpp_logger_stub.hpp`。

### 3. 编译

```bash
cd trac_ik/trac_ik_lib
mkdir build
cd build

# Windows (使用 vcpkg)
cmake .. -DCMAKE_BUILD_TYPE=Release ^
         -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
         -DUSE_ROS=0

# Linux
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ROS=0

# 编译
cmake --build . --config Release
```

### 4. 集成到项目

#### Visual Studio 项目设置

1. **包含目录**：
   ```
   ..\trac_ik\trac_ik_lib\include
   ```

2. **库目录**：
   ```
   ..\trac_ik\trac_ik_lib\build\Release
   C:\vcpkg\installed\x64-windows\lib
   ```

3. **链接库**：
   ```
   trac_ik.lib
   nlopt.lib
   orocos-kdl.lib
   ```

### 5. 使用示例

```cpp
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

// 假设你已经有了 KDL::Chain
KDL::Chain robot_chain;  // 你的机器人链

// 设置关节限位
KDL::JntArray q_min(6), q_max(6);
q_min(0) = -3.14; q_max(0) = 3.14;
q_min(1) = -2.09; q_max(1) = 2.09;
// ... 设置其他关节

// 创建 TRAC-IK 求解器
TRAC_IK::TRAC_IK ik_solver(
    robot_chain,
    q_min,
    q_max,
    0.005,        // timeout (秒)
    1e-5,         // error tolerance
    TRAC_IK::Speed  // 求解类型
);

// 求解 IK
KDL::Frame target_pose;  // 目标位姿
KDL::JntArray q_init(6); // 初始关节值（种子）
KDL::JntArray q_result(6); // 输出结果

int result = ik_solver.CartToJnt(q_init, target_pose, q_result);

if (result >= 0) {
    // 求解成功！
    // q_result 包含满足限位的关节值
} else {
    // 求解失败
}
```

## 常见问题

### Q: 编译时找不到 rclcpp
A: 确保已经修改了源码，使用 `rclcpp_logger_stub.hpp` 替换 `rclcpp/rclcpp.hpp`。

### Q: 链接错误，找不到 NLopt
A: 确保安装了 NLopt 并正确设置了库路径。

### Q: 可以使用 URDF 构造函数吗？
A: 不可以。非 ROS 环境下只支持使用 `KDL::Chain` 的构造函数。你需要手动构建 `KDL::Chain`。

### Q: 日志输出在哪里？
A: 日志会输出到 `stderr`（标准错误流）。在 Visual Studio 中会显示在输出窗口。

## 下一步

1. ✅ 完成源码修改
2. ✅ 编译 TRAC-IK 库
3. ✅ 集成到 X-Robot 项目
4. ✅ 替换 `ChainIkSolverPos_LMA` 为 `TRAC_IK::TRAC_IK`
5. ✅ 测试验证

详细的集成步骤请参考 `BUILD_STANDALONE.md`。

