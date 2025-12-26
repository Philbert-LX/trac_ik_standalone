/**
 * TRAC-IK 基本使用示例
 * 
 * 这个示例展示了如何使用 TRAC-IK 进行基本的逆运动学求解
 */

#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::cout << "=== TRAC-IK 基本使用示例 ===" << std::endl;
    
    // ==========================================
    // 步骤 1: 构建机器人链
    // ==========================================
    std::cout << "\n1. 构建机器人链..." << std::endl;
    
    KDL::Chain chain;
    
    // 添加 6 个旋转关节（简化示例，实际需要根据机器人模型）
    // 这里使用 UR5 的简化 DH 参数
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
            std::get<0>(dh),  // a (连杆长度)
            std::get<1>(dh),  // alpha (连杆扭转角)
            std::get<2>(dh),  // d (连杆偏移)
            std::get<3>(dh)   // theta (关节角度)
        );
        chain.addSegment(KDL::Segment(joint, frame));
    }
    
    std::cout << "   链已构建，自由度: " << chain.getNrOfJoints() << std::endl;
    
    // ==========================================
    // 步骤 2: 设置关节限位
    // ==========================================
    std::cout << "\n2. 设置关节限位..." << std::endl;
    
    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);
    
    // UR5 关节限位（弧度）
    q_min(0) = -M_PI; q_max(0) = M_PI;  // ±180°
    q_min(1) = -M_PI; q_max(1) = M_PI;
    q_min(2) = -M_PI; q_max(2) = M_PI;
    q_min(3) = -M_PI; q_max(3) = M_PI;
    q_min(4) = -M_PI; q_max(4) = M_PI;
    q_min(5) = -M_PI; q_max(5) = M_PI;
    
    std::cout << "   关节限位已设置" << std::endl;
    
    // ==========================================
    // 步骤 3: 创建 TRAC-IK 求解器
    // ==========================================
    std::cout << "\n3. 创建 TRAC-IK 求解器..." << std::endl;
    
    TRAC_IK::TRAC_IK ik_solver(
        chain,           // 机器人链
        q_min,           // 关节下限
        q_max,           // 关节上限
        0.005,           // 超时时间（5ms）
        1e-5,            // 误差容差
        TRAC_IK::Speed   // 求解模式：速度优先
    );
    
    std::cout << "   IK 求解器已创建" << std::endl;
    
    // ==========================================
    // 步骤 4: 定义目标位姿
    // ==========================================
    std::cout << "\n4. 定义目标位姿..." << std::endl;
    
    KDL::Frame target_pose;
    
    // 设置位置 (x, y, z) 单位：米
    target_pose.p = KDL::Vector(0.4, 0.2, 0.3);
    
    // 设置姿态 (roll, pitch, yaw) 单位：弧度
    target_pose.M = KDL::Rotation::RPY(0.1, 0.2, 0.3);
    
    std::cout << "   目标位置: (" 
              << target_pose.p.x() << ", " 
              << target_pose.p.y() << ", " 
              << target_pose.p.z() << ")" << std::endl;
    
    // ==========================================
    // 步骤 5: 设置初始关节值（种子）
    // ==========================================
    std::cout << "\n5. 设置初始关节值（种子）..." << std::endl;
    
    KDL::JntArray q_init(6);
    q_init(0) = 0.0;
    q_init(1) = 0.0;
    q_init(2) = 0.0;
    q_init(3) = 0.0;
    q_init(4) = 0.0;
    q_init(5) = 0.0;
    
    std::cout << "   初始关节值已设置" << std::endl;
    
    // ==========================================
    // 步骤 6: 求解 IK
    // ==========================================
    std::cout << "\n6. 求解 IK..." << std::endl;
    
    KDL::JntArray q_result(6);
    int result = ik_solver.CartToJnt(q_init, target_pose, q_result);
    
    // ==========================================
    // 步骤 7: 检查结果
    // ==========================================
    std::cout << "\n7. 结果分析..." << std::endl;
    
    if (result >= 0) {
        std::cout << "   ✓ IK 求解成功！" << std::endl;
        std::cout << "   找到 " << result << " 个解" << std::endl;
        
        std::cout << "\n   关节角度（第一个解）：" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 6; ++i) {
            double deg = q_result(i) * 180.0 / M_PI;
            std::cout << "     关节 " << (i+1) << ": " 
                      << std::setw(8) << q_result(i) << " rad  (" 
                      << std::setw(8) << deg << "°)" << std::endl;
        }
        
        // ==========================================
        // 步骤 8: 验证结果（可选）
        // ==========================================
        std::cout << "\n8. 验证结果..." << std::endl;
        
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::Frame verify_pose;
        fk_solver.JntToCart(q_result, verify_pose);
        
        // 计算位置误差
        KDL::Vector pos_error = target_pose.p - verify_pose.p;
        double pos_error_norm = pos_error.Norm();
        
        // 计算姿态误差
        KDL::Rotation rot_error = target_pose.M * verify_pose.M.Inverse();
        KDL::Vector rot_error_axis;
        double rot_error_angle;
        rot_error.GetRotAngle(rot_error_axis, rot_error_angle);
        
        std::cout << "   位置误差: " << pos_error_norm * 1000.0 << " mm" << std::endl;
        std::cout << "   姿态误差: " << rot_error_angle * 180.0 / M_PI << " deg" << std::endl;
        
        if (pos_error_norm < 1e-4 && rot_error_angle < 1e-3) {
            std::cout << "   ✓ 验证通过！" << std::endl;
        } else {
            std::cout << "   ⚠ 验证失败，误差较大" << std::endl;
        }
        
    } else {
        std::cout << "   ✗ IK 求解失败" << std::endl;
        std::cout << "   错误码: " << result << std::endl;
        
        switch (result) {
            case -1:
                std::cout << "   原因: TRAC-IK 未正确初始化" << std::endl;
                break;
            case -2:
                std::cout << "   原因: 链或限位无效" << std::endl;
                break;
            case -3:
                std::cout << "   原因: 未找到解（可能目标不可达或超时）" << std::endl;
                break;
            default:
                std::cout << "   原因: 未知错误" << std::endl;
                break;
        }
        
        std::cout << "\n   建议：" << std::endl;
        std::cout << "   - 检查目标位姿是否在工作空间内" << std::endl;
        std::cout << "   - 尝试不同的初始关节值" << std::endl;
        std::cout << "   - 增加超时时间" << std::endl;
        std::cout << "   - 放宽误差容差" << std::endl;
    }
    
    std::cout << "\n=== 示例完成 ===" << std::endl;
    
    return (result >= 0) ? 0 : 1;
}

