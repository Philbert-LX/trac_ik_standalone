/**
 * TRAC-IK 获取多个解的示例
 * 
 * 关键点：
 * 1. 不能使用 Speed 模式（找到第一个解就返回）
 * 2. 使用 Distance、Manip1、Manip2 或 Manip3 模式
 * 3. 增加超时时间可以获得更多解
 * 4. 调用 CartToJnt() 后使用 getSolutions() 获取所有解
 */

#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <iostream>
#include <iomanip>

int main() {
    // 1. 创建机器人链（这里需要根据你的实际机器人模型构建）
    KDL::Chain chain;
    // ... 添加关节和连杆 ...
    
    // 2. 设置关节限位
    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);
    
    // 设置关节限位（示例值）
    q_min(0) = -3.14; q_max(0) = 3.14;
    q_min(1) = -2.09; q_max(1) = 2.09;
    q_min(2) = -3.14; q_max(2) = 3.14;
    q_min(3) = -3.14; q_max(3) = 3.14;
    q_min(4) = -2.09; q_max(4) = 2.09;
    q_min(5) = -3.14; q_max(5) = 3.14;
    
    // 3. 创建 TRAC-IK 求解器
    // ⚠️ 重要：不能使用 Speed 模式！Speed 模式找到第一个解就返回
    // 使用 Distance、Manip1、Manip2 或 Manip3 模式才能获取多个解
    
    // 选项 1：距离优先模式（推荐用于获取多解）
    TRAC_IK::TRAC_IK ik_solver(
        chain,
        q_min,
        q_max,
        0.05,              // ⚠️ 增加超时时间（50ms）以获得更多解
        1e-5,              // 误差容差
        TRAC_IK::Distance  // 距离优先模式：会收集所有解并选择最接近 seed 的
    );
    
    // 选项 2：操作度优化模式（也可以获取多解）
    // TRAC_IK::TRAC_IK ik_solver(chain, q_min, q_max, 0.05, 1e-5, TRAC_IK::Manip1);
    
    // 4. 定义目标位姿
    KDL::Frame target_pose;
    target_pose.p = KDL::Vector(0.5, 0.2, 0.3);  // 位置
    target_pose.M = KDL::Rotation::RPY(0.1, 0.2, 0.3);  // 姿态
    
    // 5. 设置初始关节值（种子）
    KDL::JntArray q_init(6);
    q_init(0) = 0.0;
    q_init(1) = 0.0;
    q_init(2) = 0.0;
    q_init(3) = 0.0;
    q_init(4) = 0.0;
    q_init(5) = 0.0;
    
    // 6. 求解 IK
    KDL::JntArray q_out(6);
    int num_solutions = ik_solver.CartToJnt(q_init, target_pose, q_out);
    
    std::cout << "CartToJnt() 返回的解数量: " << num_solutions << std::endl;
    std::cout << "q_out 包含最优解（根据求解模式选择）" << std::endl;
    std::cout << std::endl;
    
    // 7. 获取所有找到的解
    std::vector<KDL::JntArray> all_solutions;
    std::vector<std::pair<double, uint>> errors;
    
    if (ik_solver.getSolutions(all_solutions, errors)) {
        std::cout << "=== 找到 " << all_solutions.size() << " 个解 ===" << std::endl;
        std::cout << std::endl;
        
        // 打印所有解
        for (size_t i = 0; i < all_solutions.size(); ++i) {
            std::cout << "解 #" << i << " (误差/评分: " << std::fixed 
                      << std::setprecision(6) << errors[i].first << "):" << std::endl;
            
            const KDL::JntArray& sol = all_solutions[i];
            std::cout << "  关节值: [";
            for (unsigned int j = 0; j < sol.rows(); ++j) {
                std::cout << std::setw(8) << std::setprecision(4) << sol(j);
                if (j < sol.rows() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            std::cout << std::endl;
        }
        
        // 说明：errors 已经根据求解模式排序
        // - Distance/Speed: 按误差从小到大排序（误差越小越好）
        // - Manip1/2/3: 按操作度从大到小排序（操作度越大越好）
        
        std::cout << "最优解索引: " << errors[0].second << std::endl;
        std::cout << "最优解就是 q_out" << std::endl;
        
    } else {
        std::cout << "无法获取解（求解失败或未找到解）" << std::endl;
    }
    
    return 0;
}

