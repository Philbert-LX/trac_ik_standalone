# TRAC-IK API 使用示例

## 📖 目录

1. [基本使用示例](#基本使用示例)
2. [构建 KDL::Chain](#构建-kdlchain)
3. [设置关节限位](#设置关节限位)
4. [不同求解模式](#不同求解模式)
5. [完整工作示例](#完整工作示例)
6. [错误处理](#错误处理)
7. [性能优化建议](#性能优化建议)

## 基本使用示例

### 示例 1：最简单的 IK 求解

```cpp
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <iostream>

int main() {
    // 1. 创建机器人链（这里以 6 自由度为例）
    KDL::Chain chain;
    
    // 添加关节和连杆
    // 注意：实际使用时需要根据你的机器人模型添加
    // chain.addSegment(KDL::Segment(...));
    
    // 2. 设置关节限位
    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);
    
    // 设置每个关节的限位（单位：弧度）
    q_min(0) = -3.14; q_max(0) = 3.14;  // 关节 1
    q_min(1) = -2.09; q_max(1) = 2.09;  // 关节 2
    q_min(2) = -3.14; q_max(2) = 3.14;  // 关节 3
    q_min(3) = -3.14; q_max(3) = 3.14;  // 关节 4
    q_min(4) = -2.09; q_max(4) = 2.09;  // 关节 5
    q_min(5) = -3.14; q_max(5) = 3.14;  // 关节 6
    
    // 3. 创建 TRAC-IK 求解器
    TRAC_IK::TRAC_IK ik_solver(
        chain,           // 机器人链
        q_min,           // 关节下限
        q_max,           // 关节上限
        0.005,           // 超时时间（秒）
        1e-5,            // 误差容差
        TRAC_IK::Speed   // 求解模式：速度优先
    );
    
    // 4. 定义目标位姿
    KDL::Frame target_pose;
    target_pose.p = KDL::Vector(0.5, 0.2, 0.3);  // 位置 (x, y, z)
    target_pose.M = KDL::Rotation::RPY(0.1, 0.2, 0.3);  // 姿态 (roll, pitch, yaw)
    
    // 5. 设置初始关节值（种子）
    KDL::JntArray q_init(6);
    q_init(0) = 0.0;
    q_init(1) = 0.0;
    q_init(2) = 0.0;
    q_init(3) = 0.0;
    q_init(4) = 0.0;
    q_init(5) = 0.0;
    
    // 6. 求解 IK
    KDL::JntArray q_result(6);
    int result = ik_solver.CartToJnt(q_init, target_pose, q_result);
    
    // 7. 检查结果
    if (result >= 0) {
        std::cout << "IK 求解成功！找到 " << result << " 个解" << std::endl;
        std::cout << "关节角度（弧度）：" << std::endl;
        for (int i = 0; i < 6; ++i) {
            std::cout << "  关节 " << i << ": " << q_result(i) << std::endl;
        }
    } else {
        std::cout << "IK 求解失败，错误码: " << result << std::endl;
    }
    
    return 0;
}
```

## 构建 KDL::Chain

### 示例 2：从 DH 参数构建链

```cpp
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>

KDL::Chain buildChainFromDH(const std::vector<DHParams>& dh_params) {
    KDL::Chain chain;
    
    for (const auto& dh : dh_params) {
        // 创建关节
        KDL::Joint joint(
            KDL::Joint::RotZ,  // 旋转关节（或 RotX, RotY, TransX, TransY, TransZ）
            KDL::Frame::DH(dh.a, dh.alpha, dh.d, dh.theta)
        );
        
        // 创建连杆（这里简化，实际需要设置惯性参数）
        KDL::Segment segment(
            KDL::Joint(joint),
            KDL::Frame::Identity()  // 连杆坐标系
        );
        
        chain.addSegment(segment);
    }
    
    return chain;
}

// DH 参数结构
struct DHParams {
    double a;      // 连杆长度
    double alpha;  // 连杆扭转角
    double d;      // 连杆偏移
    double theta;  // 关节角度
};

// 使用示例
void example() {
    std::vector<DHParams> dh = {
        {0.0, M_PI/2, 0.089, 0.0},
        {-0.425, 0.0, 0.0, 0.0},
        {-0.392, 0.0, 0.0, 0.0},
        {0.0, M_PI/2, 0.1093, 0.0},
        {0.0, -M_PI/2, 0.09465, 0.0},
        {0.0, 0.0, 0.0823, 0.0}
    };
    
    KDL::Chain chain = buildChainFromDH(dh);
}
```

### 示例 3：手动构建链（通用方法）

```cpp
KDL::Chain buildSimpleChain() {
    KDL::Chain chain;
    
    // 添加第一个关节（旋转关节，绕 Z 轴）
    KDL::Joint joint1(KDL::Joint::RotZ);
    KDL::Frame frame1 = KDL::Frame::DH(0.0, M_PI/2, 0.089, 0.0);
    KDL::Segment segment1(joint1, frame1);
    chain.addSegment(segment1);
    
    // 添加第二个关节
    KDL::Joint joint2(KDL::Joint::RotZ);
    KDL::Frame frame2 = KDL::Frame::DH(-0.425, 0.0, 0.0, 0.0);
    KDL::Segment segment2(joint2, frame2);
    chain.addSegment(segment2);
    
    // ... 继续添加其他关节
    
    return chain;
}
```

## 设置关节限位

### 示例 4：从 URDF 限位值设置

```cpp
void setJointLimitsFromURDF(KDL::JntArray& q_min, KDL::JntArray& q_max) {
    // 假设从 URDF 解析得到的限位值
    std::vector<double> lower_limits = {-3.14, -2.09, -3.14, -3.14, -2.09, -3.14};
    std::vector<double> upper_limits = {3.14, 2.09, 3.14, 3.14, 2.09, 3.14};
    
    int num_joints = lower_limits.size();
    q_min.resize(num_joints);
    q_max.resize(num_joints);
    
    for (int i = 0; i < num_joints; ++i) {
        q_min(i) = lower_limits[i];
        q_max(i) = upper_limits[i];
    }
}
```

### 示例 5：连续关节（无限位）

```cpp
void setJointLimitsWithContinuous(KDL::JntArray& q_min, KDL::JntArray& q_max) {
    int num_joints = 6;
    q_min.resize(num_joints);
    q_max.resize(num_joints);
    
    // 前 3 个关节有限位
    q_min(0) = -3.14; q_max(0) = 3.14;
    q_min(1) = -2.09; q_max(1) = 2.09;
    q_min(2) = -3.14; q_max(2) = 3.14;
    
    // 后 3 个关节是连续关节（无限位）
    q_min(3) = std::numeric_limits<double>::lowest();
    q_max(3) = std::numeric_limits<double>::max();
    q_min(4) = std::numeric_limits<double>::lowest();
    q_max(4) = std::numeric_limits<double>::max();
    q_min(5) = std::numeric_limits<double>::lowest();
    q_max(5) = std::numeric_limits<double>::max();
}
```

## 不同求解模式

### 示例 6：速度优先模式（Speed）

```cpp
TRAC_IK::TRAC_IK ik_solver(
    chain, q_min, q_max,
    0.005,        // 超时：5ms
    1e-5,         // 误差容差
    TRAC_IK::Speed  // 速度优先：找到第一个解就返回
);

// 适合：实时控制、快速响应场景
```

### 示例 7：距离优先模式（Distance）

```cpp
TRAC_IK::TRAC_IK ik_solver(
    chain, q_min, q_max,
    0.005,
    1e-5,
    TRAC_IK::Distance  // 距离优先：返回最接近初始值的解
);

// 适合：平滑运动、最小关节变化场景
```

### 示例 8：操作度优化模式（Manip1/2/3）

```cpp
// Manip1: 操作度乘积最大化
TRAC_IK::TRAC_IK ik_solver_manip1(
    chain, q_min, q_max,
    0.005,
    1e-5,
    TRAC_IK::Manip1
);

// Manip2: 条件数最小化（最稳定）
TRAC_IK::TRAC_IK ik_solver_manip2(
    chain, q_min, q_max,
    0.005,
    1e-5,
    TRAC_IK::Manip2
);

// Manip3: 最小奇异值最大化
TRAC_IK::TRAC_IK ik_solver_manip3(
    chain, q_min, q_max,
    0.005,
    1e-5,
    TRAC_IK::Manip3
);

// 适合：需要优化机器人操作性能的场景
```

### 示例 9：动态切换求解模式

```cpp
TRAC_IK::TRAC_IK ik_solver(chain, q_min, q_max);

// 运行时切换模式
ik_solver.SetSolveType(TRAC_IK::Speed);      // 切换到速度模式
ik_solver.SetSolveType(TRAC_IK::Distance);   // 切换到距离模式
ik_solver.SetSolveType(TRAC_IK::Manip1);     // 切换到操作度模式
```

## 完整工作示例

### 示例 10：完整的 6 自由度机器人 IK 求解

```cpp
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <iostream>
#include <iomanip>

class RobotIK {
public:
    RobotIK() {
        // 构建机器人链（UR5 示例）
        buildUR5Chain();
        
        // 设置关节限位
        setUR5Limits();
        
        // 创建 IK 求解器
        ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
            chain_,
            q_min_,
            q_max_,
            0.005,        // 5ms 超时
            1e-5,         // 1e-5 误差容差
            TRAC_IK::Speed
        );
        
        // 创建 FK 求解器（用于验证）
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    }
    
    // IK 求解
    bool solveIK(const KDL::Frame& target, const KDL::JntArray& seed, KDL::JntArray& result) {
        int rc = ik_solver_->CartToJnt(seed, target, result);
        
        if (rc >= 0) {
            // 验证结果（可选）
            KDL::Frame verify_pose;
            fk_solver_->JntToCart(result, verify_pose);
            
            // 计算位置误差
            double pos_error = (target.p - verify_pose.p).Norm();
            double rot_error = KDL::diff(target.M, verify_pose.M).Norm();
            
            std::cout << "位置误差: " << pos_error << " m" << std::endl;
            std::cout << "姿态误差: " << rot_error << " rad" << std::endl;
            
            return true;
        }
        return false;
    }
    
    // 获取多个解
    bool getMultipleSolutions(const KDL::Frame& target, const KDL::JntArray& seed,
                              std::vector<KDL::JntArray>& solutions) {
        int rc = ik_solver_->CartToJnt(seed, target, KDL::JntArray(6));
        if (rc >= 0) {
            return ik_solver_->getSolutions(solutions);
        }
        return false;
    }
    
private:
    void buildUR5Chain() {
        // UR5 的简化 DH 参数
        std::vector<std::tuple<double, double, double, double>> dh_params = {
            {0.0, M_PI/2, 0.089159, 0.0},      // 关节 1
            {-0.425, 0.0, 0.0, 0.0},           // 关节 2
            {-0.39225, 0.0, 0.0, 0.0},         // 关节 3
            {0.0, M_PI/2, 0.10915, 0.0},       // 关节 4
            {0.0, -M_PI/2, 0.09465, 0.0},      // 关节 5
            {0.0, 0.0, 0.0823, 0.0}            // 关节 6
        };
        
        for (const auto& dh : dh_params) {
            KDL::Joint joint(KDL::Joint::RotZ);
            KDL::Frame frame = KDL::Frame::DH(
                std::get<0>(dh),  // a
                std::get<1>(dh),  // alpha
                std::get<2>(dh),  // d
                std::get<3>(dh)   // theta
            );
            chain_.addSegment(KDL::Segment(joint, frame));
        }
    }
    
    void setUR5Limits() {
        q_min_.resize(6);
        q_max_.resize(6);
        
        // UR5 关节限位（弧度）
        q_min_(0) = -3.14159; q_max_(0) = 3.14159;  // ±180°
        q_min_(1) = -3.14159; q_max_(1) = 3.14159;  // ±180°
        q_min_(2) = -3.14159; q_max_(2) = 3.14159;  // ±180°
        q_min_(3) = -3.14159; q_max_(3) = 3.14159;  // ±180°
        q_min_(4) = -3.14159; q_max_(4) = 3.14159;  // ±180°
        q_min_(5) = -3.14159; q_max_(5) = 3.14159;  // ±180°
    }
    
    KDL::Chain chain_;
    KDL::JntArray q_min_, q_max_;
    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

int main() {
    RobotIK robot;
    
    // 定义目标位姿
    KDL::Frame target;
    target.p = KDL::Vector(0.4, 0.2, 0.3);  // 位置
    target.M = KDL::Rotation::RPY(0.0, 0.0, 0.0);  // 姿态
    
    // 设置初始种子
    KDL::JntArray seed(6);
    seed(0) = 0.0;
    seed(1) = 0.0;
    seed(2) = 0.0;
    seed(3) = 0.0;
    seed(4) = 0.0;
    seed(5) = 0.0;
    
    // 求解 IK
    KDL::JntArray result(6);
    if (robot.solveIK(target, seed, result)) {
        std::cout << "IK 求解成功！" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 6; ++i) {
            std::cout << "关节 " << i << ": " << result(i) 
                      << " rad (" << result(i) * 180.0 / M_PI << "°)" << std::endl;
        }
    } else {
        std::cout << "IK 求解失败" << std::endl;
    }
    
    return 0;
}
```

## 错误处理

### 示例 11：完整的错误处理

```cpp
int solveIKWithErrorHandling(TRAC_IK::TRAC_IK& ik_solver,
                             const KDL::JntArray& seed,
                             const KDL::Frame& target,
                             KDL::JntArray& result) {
    int rc = ik_solver.CartToJnt(seed, target, result);
    
    switch (rc) {
        case -1:
            std::cerr << "错误：TRAC-IK 未正确初始化" << std::endl;
            break;
        case -2:
            std::cerr << "错误：链或限位无效" << std::endl;
            break;
        case -3:
            std::cerr << "警告：未找到解（超时或不可达）" << std::endl;
            break;
        default:
            if (rc > 0) {
                std::cout << "成功：找到 " << rc << " 个解" << std::endl;
            }
            break;
    }
    
    return rc;
}

// 使用示例
void example() {
    TRAC_IK::TRAC_IK ik_solver(chain, q_min, q_max);
    KDL::JntArray seed(6), result(6);
    KDL::Frame target;
    
    int rc = solveIKWithErrorHandling(ik_solver, seed, target, result);
    
    if (rc >= 0) {
        // 使用结果
        // ...
    } else {
        // 处理错误
        // 可以尝试：
        // 1. 调整目标位姿
        // 2. 使用不同的种子值
        // 3. 增加超时时间
        // 4. 放宽误差容差
    }
}
```

### 示例 12：重试机制

```cpp
bool solveIKWithRetry(TRAC_IK::TRAC_IK& ik_solver,
                      const KDL::Frame& target,
                      KDL::JntArray& result,
                      int max_retries = 10) {
    KDL::JntArray seed(6);
    
    for (int i = 0; i < max_retries; ++i) {
        // 生成随机种子
        for (int j = 0; j < 6; ++j) {
            seed(j) = (rand() / double(RAND_MAX) - 0.5) * 2.0 * M_PI;
        }
        
        int rc = ik_solver.CartToJnt(seed, target, result);
        if (rc >= 0) {
            return true;
        }
    }
    
    return false;
}
```

## 性能优化建议

### 示例 13：性能优化技巧

```cpp
// 1. 使用合适的超时时间
TRAC_IK::TRAC_IK ik_solver_fast(
    chain, q_min, q_max,
    0.001,  // 1ms - 快速响应
    1e-4,   // 稍大的误差容差
    TRAC_IK::Speed
);

TRAC_IK::TRAC_IK ik_solver_precise(
    chain, q_min, q_max,
    0.01,   // 10ms - 更精确
    1e-6,   // 更小的误差容差
    TRAC_IK::Distance
);

// 2. 选择合适的种子值
// - 使用上一次的解作为种子（平滑运动）
// - 使用工作空间中心位置
// - 使用目标位置附近的随机值

// 3. 批量求解时重用求解器
void batchSolve(const std::vector<KDL::Frame>& targets) {
    TRAC_IK::TRAC_IK ik_solver(chain, q_min, q_max);
    KDL::JntArray seed(6), result(6);
    
    for (const auto& target : targets) {
        ik_solver.CartToJnt(seed, target, result);
        // 使用当前解作为下一个种子
        seed = result;
    }
}

// 4. 使用多个解时获取所有解
void getAllSolutions(TRAC_IK::TRAC_IK& ik_solver,
                     const KDL::Frame& target,
                     const KDL::JntArray& seed) {
    KDL::JntArray dummy(6);
    int rc = ik_solver.CartToJnt(seed, target, dummy);
    
    if (rc > 0) {
        std::vector<KDL::JntArray> solutions;
        std::vector<std::pair<double, uint>> errors;
        
        if (ik_solver.getSolutions(solutions, errors)) {
            std::cout << "找到 " << solutions.size() << " 个解" << std::endl;
            
            // 按误差排序
            for (size_t i = 0; i < solutions.size(); ++i) {
                std::cout << "解 " << i << " 误差: " << errors[i].first << std::endl;
            }
        }
    }
}
```

## 常见使用场景

### 示例 14：轨迹规划中的 IK

```cpp
class TrajectoryPlanner {
public:
    bool planTrajectory(const std::vector<KDL::Frame>& waypoints,
                       std::vector<KDL::JntArray>& joint_trajectory) {
        KDL::JntArray current_joints(6);
        // 初始化当前关节值
        
        for (const auto& waypoint : waypoints) {
            KDL::JntArray next_joints(6);
            
            if (ik_solver_->CartToJnt(current_joints, waypoint, next_joints) >= 0) {
                joint_trajectory.push_back(next_joints);
                current_joints = next_joints;  // 更新当前值
            } else {
                std::cerr << "无法到达路径点" << std::endl;
                return false;
            }
        }
        
        return true;
    }
    
private:
    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
};
```

### 示例 15：实时控制循环

```cpp
void controlLoop() {
    TRAC_IK::TRAC_IK ik_solver(chain, q_min, q_max, 0.001, 1e-4, TRAC_IK::Speed);
    KDL::JntArray current_joints(6), target_joints(6);
    KDL::Frame target_pose;
    
    while (running_) {
        // 获取目标位姿
        getTargetPose(target_pose);
        
        // 快速 IK 求解
        int rc = ik_solver.CartToJnt(current_joints, target_pose, target_joints);
        
        if (rc >= 0) {
            // 发送关节命令
            sendJointCommands(target_joints);
            current_joints = target_joints;
        } else {
            // 处理失败情况
            handleIKFailure();
        }
        
        // 控制循环延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
```

## 📚 更多资源

- [TRAC-IK 论文](https://www.researchgate.net/publication/282852814_TRAC-IK_An_Open-Source_Library_for_Improved_Solving_of_Generic_Inverse_Kinematics)
- [KDL 文档](https://www.orocos.org/kdl.html)
- [完整编译指南](COMPILE_GUIDE.md)

