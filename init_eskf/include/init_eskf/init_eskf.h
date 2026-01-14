/*******************************************************
 * Init-ESKF: Error State Kalman Filter for Initialization
 * 
 * 实现基于误差状态卡尔曼滤波器的初始化系统
 * 参考 OpenVINS 设计思路
 *******************************************************/

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>
#include <map>
#include <mutex>
#include "init_eskf/utility.h"

namespace init_eskf {

// 使用 Eigen 命名空间中的类型别名
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;

/**
 * @brief Init-ESKF: 用于初始化的误差状态卡尔曼滤波器
 * 
 * 状态向量 (15维):
 * - 位置误差: δp (3D)
 * - 速度误差: δv (3D)  
 * - 姿态误差: δθ (3D)
 * - 加速度计偏置: ba (3D)
 * - 陀螺仪偏置: bg (3D)
 * 
 * 名义状态:
 * - 位置: p (3D)
 * - 速度: v (3D)
 * - 姿态四元数: q (4D)
 * - 加速度计偏置: ba (3D)
 * - 陀螺仪偏置: bg (3D)
 */
class InitESKF {
public:
    // 状态维度
    static constexpr int STATE_DIM = 15;
    
    // 名义状态结构
    struct NominalState {
        Eigen::Vector3d p;      // 位置 (m)
        Eigen::Vector3d v;      // 速度 (m/s)
        Eigen::Quaterniond q;   // 姿态四元数 (G to I)
        Eigen::Vector3d ba;     // 加速度计偏置 (m/s^2)
        Eigen::Vector3d bg;     // 陀螺仪偏置 (rad/s)
        
        NominalState() : p(Eigen::Vector3d::Zero()), 
                        v(Eigen::Vector3d::Zero()),
                        q(Eigen::Quaterniond::Identity()),
                        ba(Eigen::Vector3d::Zero()),
                        bg(Eigen::Vector3d::Zero()) {}
    };
    
    InitESKF();
    ~InitESKF();
    
    /**
     * @brief 初始化 ESKF
     * @param gravity_mag 重力大小 (m/s^2)
     * @param acc_noise 加速度计噪声标准差
     * @param gyr_noise 陀螺仪噪声标准差
     * @param acc_bias_noise 加速度计偏置随机游走噪声
     * @param gyr_bias_noise 陀螺仪偏置随机游走噪声
     */
    void initialize(double gravity_mag = 9.81,
                   double acc_noise = 0.01,
                   double gyr_noise = 0.001,
                   double acc_bias_noise = 1e-5,
                   double gyr_bias_noise = 1e-6);
    
    /**
     * @brief 使用静态对齐初始化姿态（重力对齐）
     * 收集前几秒的加速度计读数，计算平均重力方向，初始化姿态四元数
     * @param acc_samples 加速度计样本列表 (时间戳, 加速度)
     * @param gyr_samples 陀螺仪样本列表 (时间戳, 角速度)
     * @param static_duration 静态对齐持续时间 (s)，默认 2.0 秒
     * @param max_acc_variance 最大加速度方差阈值，用于检测静止状态
     * @return true 如果成功初始化姿态
     */
    bool initializeWithGravityAlignment(
        const std::vector<std::pair<double, Eigen::Vector3d>> &acc_samples,
        const std::vector<std::pair<double, Eigen::Vector3d>> &gyr_samples,
        double static_duration = 2.0,
        double max_acc_variance = 0.1);
    
    /**
     * @brief IMU 积分/预测步骤
     * @param timestamp 当前时间戳 (s)
     * @param acc 加速度计测量值 (m/s^2, IMU frame)
     * @param gyr 陀螺仪测量值 (rad/s, IMU frame)
     */
    void propagate(double timestamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    
    /**
     * @brief TOF 高度更新 (1D 量测)
     * @param timestamp 时间戳 (s)
     * @param tof_height TOF 测量的高度 (m, 从数据包真值读取)
     * @param tof_noise TOF 测量噪声标准差 (m)
     */
    void updateTOF(double timestamp, double tof_height, double tof_noise = 0.05);
    
    /**
     * @brief 光流速度更新 (2D 量测)
     * @param timestamp 时间戳 (s)
     * @param flow_velocity 光流速度 (m/s, 在相机坐标系下的 x-y 平面)
     * @param flow_noise 光流测量噪声标准差 (m/s)
     */
    void updateFlow(double timestamp, const Eigen::Vector2d &flow_velocity, double flow_noise = 0.1);
    
    /**
     * @brief 零速度更新 (ZUPT - Zero Velocity Update)
     * @param timestamp 时间戳 (s)
     * @param zupt_noise 零速度测量噪声标准差 (m/s)，用于控制约束强度
     * 
     * 在静止期间（如前两秒），强制速度保持为0，用于修正姿态和偏置估计
     */
    void updateZUPT(double timestamp, double zupt_noise = 0.01);
    
    /**
     * @brief 检测系统是否处于静止状态（基于IMU数据）
     * @param acc 当前加速度计读数 (m/s^2)
     * @param gyr 当前陀螺仪读数 (rad/s)
     * @param acc_variance_threshold 加速度方差阈值，超过则认为在运动
     * @param gyr_variance_threshold 角速度方差阈值，超过则认为在运动
     * @return true 如果检测到静止状态
     * 
     * 使用滑动窗口统计加速度和角速度的方差来判断是否静止
     */
    bool detectStaticState(const Eigen::Vector3d &acc, 
                           const Eigen::Vector3d &gyr,
                           double acc_variance_threshold = 0.5,
                           double gyr_variance_threshold = 0.1) const;
    
    /**
     * @brief 检查状态是否收敛
     * @param position_threshold 位置收敛阈值 (m)
     * @param velocity_threshold 速度收敛阈值 (m/s)
     * @param orientation_threshold 姿态收敛阈值 (rad)
     * @return true 如果状态已收敛
     */
    bool checkConvergence(double position_threshold = 0.1,
                         double velocity_threshold = 0.1,
                         double orientation_threshold = 0.05) const;
    
    /**
     * @brief 获取当前名义状态
     */
    NominalState getState() const { return x_nominal_; }
    
    /**
     * @brief 获取当前误差状态协方差矩阵
     */
    Matrix15d getCovariance() const { return P_; }
    
    /**
     * @brief 获取当前时间戳
     */
    double getTimestamp() const { return timestamp_; }
    
    /**
     * @brief 重置滤波器
     */
    void reset();
    
    /**
     * @brief 设置位置（用于位置重置）
     * @param position 新的位置 (m)
     */
    void setPosition(const Eigen::Vector3d &position);
    
    /**
     * @brief 设置速度（用于速度重置）
     * @param velocity 新的速度 (m/s)
     */
    void setVelocity(const Eigen::Vector3d &velocity);
    
    /**
     * @brief 注入误差状态到名义状态（ESKF 特性）
     */
    void injectErrorState();
    
private:
    // 名义状态
    NominalState x_nominal_;
    
    // 误差状态协方差
    Matrix15d P_;
    
    // 当前时间戳
    double timestamp_;
    double last_timestamp_;
    
    // 噪声参数
    double gravity_mag_;
    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;
    
    // 初始化标志
    bool initialized_;
    bool gravity_aligned_;  // 是否已完成重力对齐
    
    // 静止检测滑动窗口（用于自动ZUPT）
    mutable std::vector<Eigen::Vector3d> acc_window_;  // 加速度滑动窗口
    mutable std::vector<Eigen::Vector3d> gyr_window_;  // 角速度滑动窗口
    static constexpr size_t STATIC_DETECTION_WINDOW_SIZE = 50;  // 窗口大小（约0.25秒@200Hz）
    
    // 互斥锁
    mutable std::mutex mutex_;
    
    // 辅助函数
    /**
     * @brief 计算状态转移矩阵 F
     */
    Matrix15d computeStateTransitionMatrix(const Eigen::Vector3d &acc, 
                                                   const Eigen::Vector3d &gyr,
                                                   double dt) const;
    
    /**
     * @brief 计算过程噪声协方差矩阵 Q
     */
    Matrix15d computeProcessNoiseMatrix(double dt) const;
    
    /**
     * @brief 从旋转向量转换为旋转矩阵（用于误差状态）
     */
    Eigen::Matrix3d expMap(const Eigen::Vector3d &phi) const;
    
    /**
     * @brief 从旋转矩阵转换为旋转向量
     */
    Eigen::Vector3d logMap(const Eigen::Matrix3d &R) const;
};

} // namespace init_eskf
