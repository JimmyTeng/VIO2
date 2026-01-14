/*******************************************************
 * Init-ESKF: Error State Kalman Filter for Initialization
 * 
 * 实现基于误差状态卡尔曼滤波器的初始化系统
 *******************************************************/

#include "init_eskf/init_eskf.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace init_eskf;

InitESKF::InitESKF() 
    : gravity_mag_(9.81),
      // 噪声参数参考 EuRoC imu0/sensor.yaml
      // gyroscope_noise_density:      1.6968e-04   rad/s/√Hz
      // gyroscope_random_walk:        1.9393e-05   rad/s²/√Hz
      // accelerometer_noise_density:  2.0000e-03   m/s²/√Hz
      // accelerometer_random_walk:    3.0000e-03   m/s³/√Hz
      acc_noise_(2.0e-3),
      gyr_noise_(1.7e-4),
      acc_bias_noise_(3.0e-3),
      gyr_bias_noise_(2.0e-5),
      timestamp_(0.0), last_timestamp_(0.0), initialized_(false), 
      gravity_aligned_(false),
      acc_window_(), gyr_window_() {
    reset();
}

InitESKF::~InitESKF() {}

void InitESKF::initialize(double gravity_mag, double acc_noise, double gyr_noise,
                         double acc_bias_noise, double gyr_bias_noise) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    gravity_mag_ = gravity_mag;
    acc_noise_ = acc_noise;
    gyr_noise_ = gyr_noise;
    acc_bias_noise_ = acc_bias_noise;
    gyr_bias_noise_ = gyr_bias_noise;
    
    // 初始化名义状态
    x_nominal_.p = Eigen::Vector3d::Zero();
    x_nominal_.v = Eigen::Vector3d::Zero();
    x_nominal_.q = Eigen::Quaterniond::Identity();
    x_nominal_.ba = Eigen::Vector3d::Zero();
    x_nominal_.bg = Eigen::Vector3d::Zero();
    
    // 初始化协方差矩阵（较大的初始不确定性）
    P_ = Matrix15d::Identity();
    P_.block<3, 3>(0, 0) = 5.0 * Eigen::Matrix3d::Identity();  // 位置
    P_.block<3, 3>(3, 3) = 1.0 * Eigen::Matrix3d::Identity();   // 速度
    P_.block<3, 3>(6, 6) = 0.5 * Eigen::Matrix3d::Identity();   // 姿态
    P_.block<3, 3>(9, 9) = 0.1 * Eigen::Matrix3d::Identity();   // ba
    P_.block<3, 3>(12, 12) = 0.01 * Eigen::Matrix3d::Identity(); // bg
    
    initialized_ = true;
    // 注意：如果没有调用 initializeWithGravityAlignment，姿态仍然是 Identity
    // 这可能导致在斜坡上初始化失败
    gravity_aligned_ = false;
}

void InitESKF::propagate(double timestamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        initialize();
    }
    
    // 计算时间间隔
    if (last_timestamp_ > 0.0) {
        double dt = timestamp - last_timestamp_;
        if (dt <= 0.0 || dt > 0.1) {  // 检查时间间隔合理性
            last_timestamp_ = timestamp;
            return;
        }
        
        // 名义状态传播（IMU 积分）
        // 去除偏置后的测量值
        Eigen::Vector3d acc_unbiased = acc - x_nominal_.ba;
        Eigen::Vector3d gyr_unbiased = gyr - x_nominal_.bg;
        
        // 重力向量（在全局坐标系中，z 轴向上约定）
        Eigen::Vector3d gravity_global(0, 0, gravity_mag_);
        // 将重力转换到 IMU 坐标系
        Eigen::Vector3d gravity_imu = x_nominal_.q.inverse() * gravity_global;
        
        // 加速度（去除重力后，在机体系中）
        Eigen::Vector3d acc_body = acc_unbiased - gravity_imu;
        
        // 姿态更新（使用四元数）
        Eigen::Vector3d angle_increment = gyr_unbiased * dt;
        double angle_norm = angle_increment.norm();
        if (angle_norm > 1e-6) {
            Eigen::Quaterniond dq;
            dq.w() = cos(angle_norm / 2.0);
            dq.vec() = sin(angle_norm / 2.0) * angle_increment / angle_norm;
            x_nominal_.q = x_nominal_.q * dq;
            x_nominal_.q.normalize();
        }
        
        // 将加速度转换到全局坐标系
        Eigen::Vector3d acc_global = x_nominal_.q * acc_body;
        
        // 位置和速度更新（中值积分）
        x_nominal_.p = x_nominal_.p + x_nominal_.v * dt + 0.5 * acc_global * dt * dt;
        x_nominal_.v = x_nominal_.v + acc_global * dt;
        
        // 误差状态协方差传播
        Matrix15d F = computeStateTransitionMatrix(acc_unbiased, gyr_unbiased, dt);
        Matrix15d Q = computeProcessNoiseMatrix(dt);
        P_ = F * P_ * F.transpose() + Q;
        // 强制对称化，保证 P 矩阵的对称性和正定性
        P_ = (P_ + P_.transpose()) / 2.0;
    }
    
    last_timestamp_ = timestamp;
    timestamp_ = timestamp;
}

void InitESKF::updateTOF(double timestamp, double tof_height, double tof_noise) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) return;
    
    // TOF 量测模型: z = p_z (高度是位置的 z 分量)
    // 量测雅可比矩阵 H: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = 1.0;  // 对应位置 z 分量
    
    // 量测值
    double z = tof_height;
    
    // 预测量测值
    double h = x_nominal_.p.z();
    
    // 量测残差
    double y = z - h;
    
    // 量测噪声协方差
    double R = tof_noise * tof_noise;
    
    // 卡尔曼增益
    double S = (H * P_ * H.transpose())(0, 0) + R;
    Eigen::Matrix<double, 15, 1> K = P_ * H.transpose() / S;
    
    // 更新误差状态（这里我们直接更新名义状态，因为误差状态在 ESKF 中通常很小）
    // 对于 ESKF，我们应该更新误差状态，然后注入到名义状态
    Vector15d delta_x = K * y;
    
    // 注入误差状态到名义状态
    // TOF高度测量只观测z位置，H矩阵中只有H(0,2)=1.0，其他都是0
    // 因此只更新z位置，不更新其他状态（速度、姿态、偏置等）
    x_nominal_.p.z() += delta_x(2);  // 只更新z位置
    // 注意：虽然由于协方差矩阵P的非对角项，delta_x的其他分量可能非零，
    // 但这些更新没有物理意义，因为高度测量与这些状态无关
    
    // 更新协方差（使用对称化处理保证数值稳定性）
    Matrix15d I = Matrix15d::Identity();
    P_ = (I - K * H) * P_;
    // 强制对称化，保证 P 矩阵的对称性和正定性
    P_ = (P_ + P_.transpose()) / 2.0;
    
    timestamp_ = timestamp;
}

void InitESKF::updateFlow(double timestamp, const Eigen::Vector2d &flow_velocity, double flow_noise) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) return;
    
    // ==========================================
    // 1. 准备数据 & 坐标转换
    // ==========================================
    
    // 获取旋转矩阵 R_wb (Body -> World)
    Eigen::Matrix3d R_wb = x_nominal_.q.toRotationMatrix();
    
    // 将光流测量的机体系速度转换为世界系速度
    // 假设 flow_velocity 是真实的机体系速度 (vx, vy)
    Eigen::Vector3d v_b_meas(flow_velocity.x(), flow_velocity.y(), 0.0);
    Eigen::Vector3d v_w_meas = R_wb * v_b_meas; 
    
    // 获取当前 ESKF 估计的世界系速度
    Eigen::Vector3d v_w_est = x_nominal_.v;

    // ==========================================
    // 2. 构建量测模型
    // ==========================================
    
    // 残差 y = z - h = 测量值 - 预测值
    // 这里我们只使用 X 和 Y 轴的速度作为观测
    Eigen::Vector2d y;
    y.x() = v_w_meas.x() - v_w_est.x();
    y.y() = v_w_meas.y() - v_w_est.y();
    
    // ==========================================
    // 3. 构建 H 矩阵 (雅可比矩阵)
    // ==========================================
    // 状态向量顺序假定为: [p(0-2), v(3-5), theta(6-8), ba(9-11), bg(12-14)]
    // 观测方程线性化: y = H * delta_x + n
    // y = v_meas - v_est = v_true - (v_true + delta_v) = -delta_v
    
    Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
    
    // 3.1 对速度误差的导数: ∂y/∂δv = -I
    H(0, 3) = -1.0;
    H(1, 4) = -1.0;
    
    // 3.2 对姿态误差的导数: ∂y/∂δθ
    // 推导: v_w = R_wb * v_b
    // 扰动: v_w_true = R_wb * (I + [delta_theta]x) * v_b (如果是右乘局部误差)
    // 展开: v_w_true = v_w_est + R_wb * [delta_theta]x * v_b 
    //                 = v_w_est - R_wb * [v_b]x * delta_theta
    // 因此 ∂v/∂δθ = -R_wb * [v_b]x
    // 这等价于 -[R_wb * v_b]x = -[v_w_est]x
    // 残差 y = v_meas - v_est，故导数符号取反，H = [v_w_est]x
    
    Eigen::Matrix3d v_skew = init_eskf::Utility::skewSymmetric(v_w_est);
    // 取前两行 (对应 x, y 速度)
    H.block<2, 3>(0, 6) = v_skew.block<2, 3>(0, 0);

    // ==========================================
    // 4. 卡尔曼更新
    // ==========================================
    
    // 量测噪声协方差 (速度噪声的平方)
    Eigen::Matrix2d R_noise = flow_noise * flow_noise * Eigen::Matrix2d::Identity();
    
    // 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
    Eigen::Matrix<double, 15, 2> K = P_ * H.transpose() * (H * P_ * H.transpose() + R_noise).inverse();
    
    // 计算误差状态 delta_x
    Eigen::Matrix<double, 15, 1> delta_x = K * y;
    
    // 更新协方差矩阵 P = (I - KH)P
    Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
    P_ = (I - K * H) * P_;
    // 强制对称正定，保证数值稳定性
    P_ = (P_ + P_.transpose()) / 2.0;
    
    // ==========================================
    // 5. 状态注入 (Inject Error State)
    // ==========================================
    
    // [重要] 更新位置
    // 虽然 H 矩阵中位置项为0，但 K 矩阵的第一块 (15x2 中的 0-2行) 通常不为0，
    // 因为 P 矩阵中位置和速度存在相关性。
    x_nominal_.p.x() += delta_x(0);
    x_nominal_.p.y() += delta_x(1);
    x_nominal_.p.z() += delta_x(2);

    // [关键] 必须更新速度
    // 光流本质是速度传感器，不更新速度会导致滤波器发散
    x_nominal_.v.x() += delta_x(3);
    x_nominal_.v.y() += delta_x(4);
    x_nominal_.v.z() += delta_x(5);

    // 更新姿态
    Eigen::Vector3d delta_theta = delta_x.block<3, 1>(6, 0);
    if (delta_theta.norm() > 1e-6) {
        // 假设 delta_theta 是轴角向量
        Eigen::AngleAxisd angle_axis(delta_theta.norm(), delta_theta.normalized());
        // 更新顺序取决于你的扰动定义 (R_new = R_old * Exp(dq) 或 Exp(dq) * R_old)
        // 这里沿用常见做法 R * dq
        x_nominal_.q = x_nominal_.q * angle_axis;
        x_nominal_.q.normalize();
    }
    
    // 更新零偏
    x_nominal_.ba += delta_x.block<3, 1>(9, 0);
    x_nominal_.bg += delta_x.block<3, 1>(12, 0);

    // 更新时间戳
    timestamp_ = timestamp;
    
    // (可选) 可以在这里重置误差状态 delta_x 为 0，
    // 但通常 ESKF 框架中 delta_x 只是临时变量，不需要显式维护一个成员变量。
}

void InitESKF::updateZUPT(double timestamp, double zupt_noise) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) return;
    
    // 零速度量测模型: z = v (速度应该为0)
    // 量测雅可比矩阵 H: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] (只对速度)
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();  // 对应速度分量
    
    // 量测值（零速度）
    Eigen::Vector3d z = Eigen::Vector3d::Zero();
    
    // 预测量测值（当前速度）
    Eigen::Vector3d h = x_nominal_.v;
    
    // 量测残差
    Eigen::Vector3d y = z - h;
    
    // 量测噪声协方差
    Eigen::Matrix3d R = zupt_noise * zupt_noise * Eigen::Matrix3d::Identity();
    
    // 卡尔曼增益
    Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    
    // 更新误差状态
    Vector15d delta_x = K * y;
    
    // 注入误差状态到名义状态
    x_nominal_.p += delta_x.block<3, 1>(0, 0);
    x_nominal_.v += delta_x.block<3, 1>(3, 0);
    
    // 姿态更新（使用旋转向量）
    Eigen::Vector3d delta_theta = delta_x.block<3, 1>(6, 0);
    if (delta_theta.norm() > 1e-6) {
        Eigen::AngleAxisd angle_axis(delta_theta.norm(), delta_theta.normalized());
        x_nominal_.q = x_nominal_.q * angle_axis;
        x_nominal_.q.normalize();
    }
    
    x_nominal_.ba += delta_x.block<3, 1>(9, 0);
    x_nominal_.bg += delta_x.block<3, 1>(12, 0);
    
    // 更新协方差（使用对称化处理保证数值稳定性）
    Matrix15d I = Matrix15d::Identity();
    P_ = (I - K * H) * P_;
    // 强制对称化，保证 P 矩阵的对称性和正定性
    P_ = (P_ + P_.transpose()) / 2.0;
    
    timestamp_ = timestamp;
}

bool InitESKF::checkConvergence(double position_threshold, double velocity_threshold,
                               double orientation_threshold) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) return false;
    
    // 检查位置不确定性
    double pos_uncertainty = sqrt(P_.block<3, 3>(0, 0).trace() / 3.0);
    if (pos_uncertainty > position_threshold) {
        return false;
    }
    
    // 检查速度不确定性
    double vel_uncertainty = sqrt(P_.block<3, 3>(3, 3).trace() / 3.0);
    if (vel_uncertainty > velocity_threshold) {
        return false;
    }
    
    // 检查姿态不确定性
    double ori_uncertainty = sqrt(P_.block<3, 3>(6, 6).trace() / 3.0);
    if (ori_uncertainty > orientation_threshold) {
        return false;
    }
    
    return true;
}

void InitESKF::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    x_nominal_ = NominalState();
    P_ = Matrix15d::Identity();
    timestamp_ = 0.0;
    last_timestamp_ = 0.0;
    initialized_ = false;
    gravity_aligned_ = false;
    acc_window_.clear();
    gyr_window_.clear();
}

void InitESKF::setPosition(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    x_nominal_.p = position;
}

void InitESKF::setVelocity(const Eigen::Vector3d &velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    x_nominal_.v = velocity;
}

bool InitESKF::detectStaticState(const Eigen::Vector3d &acc, 
                                 const Eigen::Vector3d &gyr,
                                 double acc_variance_threshold,
                                 double gyr_variance_threshold) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新滑动窗口
    acc_window_.push_back(acc);
    gyr_window_.push_back(gyr);
    
    // 保持窗口大小
    if (acc_window_.size() > STATIC_DETECTION_WINDOW_SIZE) {
        acc_window_.erase(acc_window_.begin());
    }
    if (gyr_window_.size() > STATIC_DETECTION_WINDOW_SIZE) {
        gyr_window_.erase(gyr_window_.begin());
    }
    
    // 如果窗口数据不足，返回false（需要更多数据才能判断）
    if (acc_window_.size() < STATIC_DETECTION_WINDOW_SIZE / 2) {
        return false;
    }
    
    // 计算加速度均值
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    for (const auto &a : acc_window_) {
        acc_mean += a;
    }
    acc_mean /= acc_window_.size();
    
    // 计算加速度方差
    double acc_variance = 0.0;
    for (const auto &a : acc_window_) {
        acc_variance += (a - acc_mean).squaredNorm();
    }
    acc_variance /= acc_window_.size();
    
    // 计算角速度均值
    Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
    for (const auto &g : gyr_window_) {
        gyr_mean += g;
    }
    gyr_mean /= gyr_window_.size();
    
    // 计算角速度方差
    double gyr_variance = 0.0;
    for (const auto &g : gyr_window_) {
        gyr_variance += (g - gyr_mean).squaredNorm();
    }
    gyr_variance /= gyr_window_.size();
    
    // 检查是否静止：加速度和角速度的方差都应该很小
    bool is_static = (acc_variance < acc_variance_threshold) && 
                     (gyr_variance < gyr_variance_threshold);
    
    return is_static;
}

bool InitESKF::initializeWithGravityAlignment(
    const std::vector<std::pair<double, Eigen::Vector3d>> &acc_samples,
    const std::vector<std::pair<double, Eigen::Vector3d>> &gyr_samples,
    double static_duration,
    double max_acc_variance) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (acc_samples.empty()) {
        return false;
    }
    
    // 找到静态对齐时间窗口内的样本
    // 这里按照“只用最开始的一段静止数据”来做重力对齐：
    // 使用 acc_samples 中 **最早的 static_duration 秒**，而不是最后一段，
    // 对于像 EuRoC 这种起飞前有静止段、之后开始运动的序列更稳健。
    double start_time = acc_samples.front().first;
    double end_time   = start_time + static_duration;
    
    std::vector<Eigen::Vector3d> static_acc_samples;
    std::vector<Eigen::Vector3d> static_gyr_samples;
    
    for (const auto &sample : acc_samples) {
        if (sample.first >= start_time && sample.first <= end_time) {
            static_acc_samples.push_back(sample.second);
        }
    }
    
    for (const auto &sample : gyr_samples) {
        if (sample.first >= start_time && sample.first <= end_time) {
            static_gyr_samples.push_back(sample.second);
        }
    }
    
    if (static_acc_samples.empty()) {
        return false;
    }
    
    // 第一步：初步计算均值，用于离群点检测
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    for (const auto &acc : static_acc_samples) {
        acc_mean += acc;
    }
    acc_mean /= static_acc_samples.size();
    
    // 第二步：计算每个样本到均值的距离，用于离群点检测
    std::vector<double> distances;
    for (const auto &acc : static_acc_samples) {
        distances.push_back((acc - acc_mean).norm());
    }
    
    // 计算距离的中位数和 MAD (Median Absolute Deviation)
    std::vector<double> sorted_distances = distances;
    std::sort(sorted_distances.begin(), sorted_distances.end());
    double median_distance = sorted_distances[sorted_distances.size() / 2];
    
    // 计算 MAD
    std::vector<double> abs_deviations;
    for (double d : distances) {
        abs_deviations.push_back(std::abs(d - median_distance));
    }
    std::sort(abs_deviations.begin(), abs_deviations.end());
    double mad = abs_deviations[abs_deviations.size() / 2];
    
    // 使用 3 * MAD 作为离群点阈值（比标准差更鲁棒）
    double outlier_threshold = median_distance + 3.0 * mad;
    if (mad < 1e-6) {
        // 如果 MAD 太小，使用标准差作为备选
        double mean_dist = 0.0;
        for (double d : distances) {
            mean_dist += d;
        }
        mean_dist /= distances.size();
        double std_dev = 0.0;
        for (double d : distances) {
            std_dev += (d - mean_dist) * (d - mean_dist);
        }
        std_dev = std::sqrt(std_dev / distances.size());
        outlier_threshold = mean_dist + 3.0 * std_dev;
    }
    
    // 第三步：去除离群点，重新计算均值
    std::vector<Eigen::Vector3d> filtered_acc_samples;
    std::vector<Eigen::Vector3d> filtered_gyr_samples;
    
    for (size_t i = 0; i < static_acc_samples.size(); i++) {
        if (distances[i] <= outlier_threshold) {
            filtered_acc_samples.push_back(static_acc_samples[i]);
            if (i < static_gyr_samples.size()) {
                filtered_gyr_samples.push_back(static_gyr_samples[i]);
            }
        }
    }
    
    if (filtered_acc_samples.size() < static_acc_samples.size() * 0.5) {
        // 如果去除的样本超过50%，说明数据质量很差，直接返回失败
        return false;
    }
    
    if (filtered_acc_samples.empty()) {
        return false;
    }
    
    // 使用过滤后的样本重新计算均值
    acc_mean = Eigen::Vector3d::Zero();
    for (const auto &acc : filtered_acc_samples) {
        acc_mean += acc;
    }
    acc_mean /= filtered_acc_samples.size();
    
    // 计算加速度方差（用于检测静止状态）
    double acc_variance = 0.0;
    for (const auto &acc : filtered_acc_samples) {
        acc_variance += (acc - acc_mean).squaredNorm();
    }
    acc_variance /= filtered_acc_samples.size();
    
    // 调试信息
    double acc_norm = acc_mean.norm();
    // 调试信息曾用于分析静止窗口，此处恢复为静默以减少输出
    
    // 检查是否静止（加速度方差应该很小）
    if (acc_variance > max_acc_variance) {
        return false;  // 系统在运动，不适合静态对齐
    }
    
    // 检查平均加速度的模长是否接近重力
    if (std::abs(acc_norm - gravity_mag_) > 2.0) {
        return false;  // 加速度模长不合理
    }
    
    // 使用平均加速度方向来对齐重力
    // 重力在全局坐标系中指向 [0, 0, -gravity_mag_]（z轴向下）
    // 或者 [0, 0, gravity_mag_]（z轴向上），取决于约定
    // 这里使用 z 轴向上约定：重力 = [0, 0, gravity_mag_]
    Eigen::Vector3d gravity_global(0, 0, gravity_mag_);
    
    // 归一化平均加速度（在机体系中的重力方向）
    Eigen::Vector3d gravity_body = acc_mean.normalized();
    
    // 使用 FromTwoVectors 计算从机体系到全局系的旋转
    // 这会将机体系中的重力方向对齐到全局系的 z 轴
    Eigen::Quaterniond q_body_to_global = 
        Eigen::Quaterniond::FromTwoVectors(gravity_body, gravity_global.normalized());
    
    // 但是我们需要的是从全局系到机体系的旋转（G to I）
    // 所以需要取逆
    x_nominal_.q = q_body_to_global.inverse();
    x_nominal_.q.normalize();
    
    // 初始化偏置（使用过滤后的样本）
    if (!filtered_gyr_samples.empty()) {
        Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
        for (const auto &gyr : filtered_gyr_samples) {
            gyr_mean += gyr;
        }
        gyr_mean /= filtered_gyr_samples.size();
        x_nominal_.bg = gyr_mean;  // 静止时的陀螺仪读数就是偏置
    } else {
        x_nominal_.bg = Eigen::Vector3d::Zero();
    }
    
    // 加速度计偏置 = 平均加速度 - 旋转后的重力
    Eigen::Vector3d gravity_imu = x_nominal_.q.inverse() * gravity_global;
    x_nominal_.ba = acc_mean - gravity_imu;
    
    // 检查偏置的合理性（正常偏置应该在 ±0.5 m/s^2 以内）
    double ba_norm = x_nominal_.ba.norm();
    if (ba_norm > 2.0) {
        // 偏置异常大，可能是姿态估计有误，使用更保守的初始化
        // 假设偏置为 0，让滤波器在后续过程中慢慢估计
        x_nominal_.ba = Eigen::Vector3d::Zero();
        std::cerr << "[initializeWithGravityAlignment] Warning: Computed acc bias too large (" 
                  << ba_norm << " m/s^2), resetting to zero" << std::endl;
    }
    
    // 检查陀螺仪偏置的合理性（正常偏置应该在 ±0.1 rad/s 以内）
    double bg_norm = x_nominal_.bg.norm();
    if (bg_norm > 0.5) {
        x_nominal_.bg = Eigen::Vector3d::Zero();
        std::cerr << "[initializeWithGravityAlignment] Warning: Computed gyr bias too large (" 
                  << bg_norm << " rad/s), resetting to zero" << std::endl;
    }
    
    // 初始化其他状态
    x_nominal_.p = Eigen::Vector3d::Zero();
    x_nominal_.v = Eigen::Vector3d::Zero();
    
    // 初始化协方差矩阵
    P_ = Matrix15d::Identity();
    P_.block<3, 3>(0, 0) = 10.0 * Eigen::Matrix3d::Identity();  // 位置
    P_.block<3, 3>(3, 3) = 5.0 * Eigen::Matrix3d::Identity();   // 速度
    P_.block<3, 3>(6, 6) = 0.5 * Eigen::Matrix3d::Identity();   // 姿态（已对齐，不确定性较小）
    P_.block<3, 3>(9, 9) = 0.1 * Eigen::Matrix3d::Identity();   // ba
    P_.block<3, 3>(12, 12) = 0.01 * Eigen::Matrix3d::Identity(); // bg
    
    initialized_ = true;
    gravity_aligned_ = true;
    
    return true;
}

void InitESKF::injectErrorState() {
    // 在 ESKF 中，误差状态应该定期注入到名义状态
    // 这里我们在每次更新后自动注入，所以这个函数主要用于显式调用
    // 实际实现中，误差状态注入已经在 updateTOF 和 updateFlow 中完成
}

Matrix15d InitESKF::computeStateTransitionMatrix(const Eigen::Vector3d &acc,
                                                         const Eigen::Vector3d &gyr,
                                                         double dt) const {
    Matrix15d F = Matrix15d::Identity();
    
    // 获取旋转矩阵
    Eigen::Matrix3d R = x_nominal_.q.toRotationMatrix();
    
    // 重力向量
    Eigen::Vector3d g(0, 0, gravity_mag_);
    
    // F 矩阵的各个块
    // δp 对 δp: I
    // δp 对 δv: I * dt
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    
    // δv 对 δθ: -R * [acc]×
        Eigen::Matrix3d acc_skew = init_eskf::Utility::skewSymmetric(acc);
    F.block<3, 3>(3, 6) = -R * acc_skew * dt;
    
    // δv 对 δba: -R * dt
    F.block<3, 3>(3, 9) = -R * dt;
    
    // δθ 对 δθ: exp(-[ω]× * dt) ≈ I - [ω]× * dt
    Eigen::Vector3d omega = gyr * dt;
    double omega_norm = omega.norm();
    if (omega_norm > 1e-6) {
        Eigen::Matrix3d omega_skew = init_eskf::Utility::skewSymmetric(omega);
        F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - omega_skew;
    }
    
    // δθ 对 δbg: -I * dt
    F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
    
    return F;
}

Matrix15d InitESKF::computeProcessNoiseMatrix(double dt) const {
    Matrix15d Q = Matrix15d::Zero();
    
    // 过程噪声协方差
    double acc_noise_var = acc_noise_ * acc_noise_;
    double gyr_noise_var = gyr_noise_ * gyr_noise_;
    double acc_bias_noise_var = acc_bias_noise_ * acc_bias_noise_;
    double gyr_bias_noise_var = gyr_bias_noise_ * gyr_bias_noise_;
    
    // 位置噪声（来自速度积分）
    Q.block<3, 3>(0, 0) = acc_noise_var * dt * dt * dt * dt / 4.0 * Eigen::Matrix3d::Identity();
    
    // 速度噪声（来自加速度）
    Q.block<3, 3>(3, 3) = acc_noise_var * dt * dt * Eigen::Matrix3d::Identity();
    
    // 姿态噪声（来自陀螺仪）
    Q.block<3, 3>(6, 6) = gyr_noise_var * dt * dt * Eigen::Matrix3d::Identity();
    
    // 偏置噪声（随机游走）
    Q.block<3, 3>(9, 9) = acc_bias_noise_var * dt * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(12, 12) = gyr_bias_noise_var * dt * Eigen::Matrix3d::Identity();
    
    return Q;
}

Eigen::Matrix3d InitESKF::expMap(const Eigen::Vector3d &phi) const {
    double phi_norm = phi.norm();
    if (phi_norm < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Vector3d axis = phi / phi_norm;
    Eigen::AngleAxisd angle_axis(phi_norm, axis);
    return angle_axis.toRotationMatrix();
}

Eigen::Vector3d InitESKF::logMap(const Eigen::Matrix3d &R) const {
    Eigen::AngleAxisd angle_axis(R);
    return angle_axis.angle() * angle_axis.axis();
}
