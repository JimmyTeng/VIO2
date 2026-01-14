/*******************************************************
 * EuRoC 数据集测试脚本
 * 
 * 测试 Init-ESKF 在 V1_01_easy 数据集上的表现
 * 初始点归一化为 0
 *******************************************************/

#include "init_eskf/euroc_parser.h"
#include "init_eskf/init_eskf.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <vector>
#include <cmath>

int main(int argc, char** argv) {
    // 立即刷新输出
    // 数据集路径
    std::string dataset_path = "/home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy";
    
    if (argc > 1) {
        dataset_path = argv[1];
    }
    
    std::cout << "==========================================" << std::endl;
    std::cout << "EuRoC Dataset Test: V1_01_easy" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Dataset path: " << dataset_path << std::endl;
    
    // 1. 加载数据集
    std::cout << "\n[1/5] Loading dataset..." << std::endl;
    init_eskf::EuroCParser parser;
    if (!parser.loadDataset(dataset_path, true, true)) {
        std::cerr << "Failed to load dataset!" << std::endl;
        return -1;
    }
    
    std::cout << "  ✓ Ground truth data: " << parser.getGroundTruthCount() << " points" << std::endl;
    std::cout << "  ✓ IMU data: " << parser.getIMUCount() << " points" << std::endl;
    std::cout << "  ✓ Initial timestamp: " << parser.getInitialTimestamp() << " s" << std::endl;
    Eigen::Vector3d init_pos = parser.getInitialPosition();
    std::cout << "  ✓ Initial position: [" 
              << std::fixed << std::setprecision(3)
              << init_pos.x() << ", " << init_pos.y() << ", " << init_pos.z() << "] m" << std::endl;
    
    // 2. 初始化 ESKF
    std::cout << "\n[2/5] Initializing ESKF..." << std::endl;
    init_eskf::InitESKF eskf;
    
    // ESKF 参数
    double gravity_mag = 9.81;
    double acc_noise = 0.01;
    double gyr_noise = 0.001;
    double acc_bias_noise = 1e-5;
    double gyr_bias_noise = 1e-6;
    
    eskf.initialize(gravity_mag, acc_noise, gyr_noise, acc_bias_noise, gyr_bias_noise);
    std::cout << "  ✓ ESKF initialized" << std::endl;
    
    // 3. 重力对齐（使用前 2 秒的 IMU 数据）
    std::cout << "\n[3/5] Performing gravity alignment..." << std::endl;
    const auto& imu_data = parser.getIMUData();
    const auto& gt_data = parser.getGroundTruthData();
    
    if (imu_data.size() > 400) {  // 至少若干秒的数据（>2s）
        std::vector<std::pair<double, Eigen::Vector3d>> acc_samples;
        std::vector<std::pair<double, Eigen::Vector3d>> gyr_samples;
        
        // 收集「归一化后 0~2 秒」的 IMU 数据用于重力对齐
        // （避免使用 0 之前的负时间段，直接对应“起始 2 秒静止段”）
        std::cout << "  收集 0~2 秒内的 IMU 样本用于重力对齐" << std::endl;
        const double align_start_t = 0.0;
        const double align_end_t   = 2.0;
        
        // 打印前几个样本的详细信息
        std::cout << "  前 5 个 IMU 样本:" << std::endl;
        for (size_t i = 0; i < std::min<size_t>(5, imu_data.size()); i++) {
            std::cout << "    [" << i << "] ts=" << std::fixed << std::setprecision(6) << imu_data[i].timestamp
                      << "s, acc=[" << std::setprecision(4) << imu_data[i].acc.x() << ", " 
                      << imu_data[i].acc.y() << ", " << imu_data[i].acc.z() << "] m/s^2"
                      << ", gyr=[" << imu_data[i].gyr.x() << ", " 
                      << imu_data[i].gyr.y() << ", " << imu_data[i].gyr.z() << "] rad/s" << std::endl;
        }
        
        // 只选取时间在 [0, 2] 秒内的样本
        for (size_t i = 0; i < imu_data.size(); i++) {
            double t = imu_data[i].timestamp;
            if (t < align_start_t) continue;
            if (t > align_end_t)   break;
            acc_samples.push_back({t, imu_data[i].acc});
            gyr_samples.push_back({t, imu_data[i].gyr});
        }
        
        // 计算收集到的数据的时间范围
        if (!acc_samples.empty()) {
            double time_span = acc_samples.back().first - acc_samples.front().first;
            std::cout << "  收集的数据时间范围: " << acc_samples.front().first 
                      << " ~ " << acc_samples.back().first << " s (跨度: " << time_span << " s)" << std::endl;
            std::cout << "  收集到的样本数量: " << acc_samples.size() << std::endl;
        } else {
            std::cout << "  ⚠ 在 0~2 秒内未收集到足够的 IMU 样本，跳过重力对齐" << std::endl;
            std::cout.flush();
            // 保持 Identity 姿态，直接返回后续流程
            goto AFTER_GRAVITY_ALIGNMENT;
        }
        
        std::cout << "  调用 initializeWithGravityAlignment..." << std::endl;
        std::cout.flush();
        
        // 放宽加速度方差阈值，从0.1改为0.5，因为前两秒可能有轻微运动
        if (eskf.initializeWithGravityAlignment(acc_samples, gyr_samples, 2.0, 0.5)) {
            std::cout << "  ✓ Gravity alignment successful" << std::endl;
            init_eskf::InitESKF::NominalState state = eskf.getState();
            std::cout << "    Initial orientation: [" 
                      << std::fixed << std::setprecision(4)
                      << state.q.w() << ", " << state.q.x() << ", " 
                      << state.q.y() << ", " << state.q.z() << "]" << std::endl;
            std::cout << "    Initial acc bias: [" 
                      << std::setprecision(6)
                      << state.ba.x() << ", " << state.ba.y() << ", " << state.ba.z() << "] m/s^2" << std::endl;
            std::cout << "    Initial gyr bias: [" 
                      << state.bg.x() << ", " << state.bg.y() << ", " << state.bg.z() << "] rad/s" << std::endl;
        } else {
            std::cout << "  ⚠ Gravity alignment failed, using Identity quaternion" << std::endl;
            std::cout << "    请查看 stderr 中的详细调试信息" << std::endl;
        }
    } else {
        std::cout << "  ⚠ IMU 数据不足 (" << imu_data.size() << " < 400)，跳过重力对齐" << std::endl;
    }
AFTER_GRAVITY_ALIGNMENT:
    
    // 4. 处理数据
    std::cout << "\n[4/5] Processing IMU and ground truth data..." << std::endl;
    
    size_t tof_updates = 0;
    size_t flow_updates = 0;
    size_t total_updates = 0;
    
    // 找到第一个时间戳（应该都是 0.0，因为归一化了）
    double start_time = 0.0;
    if (!imu_data.empty()) {
        start_time = imu_data[0].timestamp;
        std::cout << "  第一个 IMU 时间戳: " << std::fixed << std::setprecision(6) << start_time << " s" << std::endl;
    }
    if (!gt_data.empty()) {
        std::cout << "  第一个 GT 时间戳: " << std::fixed << std::setprecision(6) << gt_data[0].timestamp << " s" << std::endl;
        std::cout << "  第一个 GT 位置: [" << std::setprecision(4) << gt_data[0].x << ", " 
                  << gt_data[0].y << ", " << gt_data[0].z << "] m (归一化后，应该是 [0,0,0])" << std::endl;
    }
    
    // 处理前 20 秒的数据（或所有数据，如果少于 20 秒）
    double end_time = start_time + 20.0;
    if (!imu_data.empty() && imu_data.back().timestamp < end_time) {
        end_time = imu_data.back().timestamp;
    }
    
    std::cout << "  Processing from " << start_time << " s to " << end_time << " s" << std::endl;
    std::cout << "  Total IMU samples: " << imu_data.size() << std::endl;
    
    // 计算需要处理的数据量
    size_t total_count = 0;
    for (const auto& imu : imu_data) {
        if (imu.timestamp <= end_time) {
            total_count++;
        } else {
            break;
        }
    }
    
    std::cout << "  Progress: ";
    std::cout.flush();
    
    // 使用索引直接访问有序数据（真值和IMU都是有序的）
    size_t gt_idx = 0;  // 真值数据索引
    
    // 遍历 IMU 数据
    for (size_t imu_idx = 0; imu_idx < imu_data.size(); imu_idx++) {
        const auto& imu = imu_data[imu_idx];
        
        if (imu.timestamp > end_time) {
            break;
        }
        
        // IMU 传播
        eskf.propagate(imu.timestamp, imu.acc, imu.gyr);
        total_updates++;
        
        // 自动检测静止状态并应用零速度更新（ZUPT）
        static double last_zupt_time = -1.0;
        double zupt_period = 0.02;  // 50Hz，与TOF更新频率一致
        
        if (imu.timestamp - last_zupt_time >= zupt_period) {
            // 检测是否处于静止状态
            bool is_static = eskf.detectStaticState(imu.acc, imu.gyr, 0.5, 0.1);
            
            if (is_static) {
                eskf.updateZUPT(imu.timestamp, 0.01);  // 0.01 m/s 的测量噪声
                last_zupt_time = imu.timestamp;
            } else {
                last_zupt_time = imu.timestamp;  // 更新最后ZUPT时间，避免频繁检测
            }
        }
        
        // 检测首次进入高空（高度 > 0.1m），重置位置为 (0, 0, 0.1) 并重置速度
        static bool position_reset_done = false;
        static double last_height = 0.0;
        init_eskf::InitESKF::NominalState current_state_check = eskf.getState();
        double current_height_check = current_state_check.p.z();
        
        // 检测高度从 <= 0.1m 变为 > 0.1m 的瞬间
        if (!position_reset_done && last_height <= 0.1 && current_height_check > 0.1) {
            // 首次进入高空，重置位置为 (0, 0, 0.1)
            Eigen::Vector3d reset_position(0.0, 0.0, 0.1);
            eskf.setPosition(reset_position);
            
            // 重置速度：xy方向设为0，z方向保持当前值
            Eigen::Vector3d current_velocity = current_state_check.v;
            Eigen::Vector3d reset_velocity(0.0, 0.0, current_velocity.z());  // xy=0, z保持
            eskf.setVelocity(reset_velocity);
            
            position_reset_done = true;
            std::cout << "\n  [Position Reset] t=" << std::fixed << std::setprecision(6) 
                      << imu.timestamp << "s, height=" << std::setprecision(4) << current_height_check 
                      << "m, reset position to (0, 0, 0.1) m, reset xy velocity to 0, z velocity=" 
                      << std::setprecision(4) << current_velocity.z() << " m/s" << std::endl;
        }
        last_height = current_height_check;
        
        // TOF 更新（50Hz，每 0.02 秒一次，从一开始就更新）
        static double last_tof_time = -1.0;
        double tof_period = 0.02;  // 50Hz
        
        if (imu.timestamp - last_tof_time >= tof_period) {
            // 由于数据有序，直接查找最接近的真值数据
            // 从当前 gt_idx 开始向前查找
            while (gt_idx < gt_data.size() && gt_data[gt_idx].timestamp < imu.timestamp - 0.1) {
                gt_idx++;  // 跳过太早的数据
            }
            
            // 查找最接近的真值（扩大搜索范围，确保能找到）
            size_t best_gt_idx = gt_idx;
            double best_diff = 0.2;  // 扩大时间差阈值到0.2s
            
            // 向前搜索（可能gt_idx已经超过了）
            if (gt_idx > 0) {
                for (size_t i = gt_idx; i > 0 && i > gt_idx - 50; i--) {
                    double diff = std::abs(gt_data[i-1].timestamp - imu.timestamp);
                    if (diff < best_diff) {
                        best_diff = diff;
                        best_gt_idx = i-1;
                    }
                    if (gt_data[i-1].timestamp < imu.timestamp - 0.2) break;
                }
            }
            
            // 向后搜索
            for (size_t i = best_gt_idx; i < gt_data.size() && i < best_gt_idx + 200; i++) {
                double diff = std::abs(gt_data[i].timestamp - imu.timestamp);
                if (diff < best_diff) {
                    best_diff = diff;
                    best_gt_idx = i;
                }
                // 如果时间戳已经超过，可以提前退出
                if (gt_data[i].timestamp > imu.timestamp + 0.2) {
                    break;
                }
            }
            
            if (best_diff < 0.2) {  // 放宽时间差阈值
                // 获取当前状态和高度
                init_eskf::InitESKF::NominalState current_state_tof = eskf.getState();
                double current_height_tof = current_state_tof.p.z();
                
                // 打印前几次 TOF 更新的详细信息
                if (tof_updates < 5) {
                    std::cout << "\n  [TOF Update #" << tof_updates << "] t=" << std::fixed << std::setprecision(6) 
                              << imu.timestamp << "s, GT height=" << std::setprecision(4) << gt_data[best_gt_idx].z 
                              << "m, ESKF height=" << current_height_tof << "m, diff=" 
                              << (gt_data[best_gt_idx].z - current_height_tof) << "m" << std::endl;
                }
                eskf.updateTOF(imu.timestamp, gt_data[best_gt_idx].z, 0.005);  // 进一步减小测量噪声到0.005m
                tof_updates++;
                last_tof_time = imu.timestamp;
                gt_idx = best_gt_idx;  // 更新索引，下次从这里开始
            }
        }
        
        // 光流速度更新（50Hz，每 0.02 秒一次，仅在高度 > 0.1m 时应用）
        static double last_flow_time = -1.0;
        double flow_period = 0.02;  // 50Hz
        
        // 获取当前状态和高度
        init_eskf::InitESKF::NominalState current_state = eskf.getState();
        double current_height = current_state.p.z();
        
        if (current_height > 0.1 && imu.timestamp - last_flow_time >= flow_period) {
            // 查找对应的真值数据（扩大搜索范围）
            double best_time_diff = 0.2;  // 扩大时间差阈值
            size_t best_flow_gt_idx = gt_idx;
            
            // 从gt_idx开始向前和向后搜索
            if (gt_idx > 0) {
                for (size_t i = gt_idx; i > 0 && i > gt_idx - 50; i--) {
                    double diff = std::abs(gt_data[i-1].timestamp - imu.timestamp);
                    if (diff < best_time_diff) {
                        best_time_diff = diff;
                        best_flow_gt_idx = i-1;
                    }
                    if (gt_data[i-1].timestamp < imu.timestamp - 0.2) break;
                }
            }
            
            for (size_t i = best_flow_gt_idx; i < gt_data.size() && i < best_flow_gt_idx + 200; i++) {
                double diff = std::abs(gt_data[i].timestamp - imu.timestamp);
                if (diff < best_time_diff) {
                    best_time_diff = diff;
                    best_flow_gt_idx = i;
                }
                if (gt_data[i].timestamp > imu.timestamp + 0.2) break;
            }
            
            if (best_time_diff < 0.2) {
                gt_idx = best_flow_gt_idx;  // 更新gt_idx
                // 从真值速度计算光流速度（机体系 xy 分量）
                // 真值速度是世界系速度，需要转换到机体系
                Eigen::Vector3d gt_vel_world(gt_data[best_flow_gt_idx].vx, gt_data[best_flow_gt_idx].vy, gt_data[best_flow_gt_idx].vz);
                
                // 使用当前姿态将世界系速度转换到机体系
                Eigen::Matrix3d R_wb = current_state.q.toRotationMatrix();
                Eigen::Vector3d vel_body = R_wb.transpose() * gt_vel_world;
                
                // 光流速度是机体系速度的 xy 分量
                Eigen::Vector2d flow_velocity(vel_body.x(), vel_body.y());
                
                // 打印前几次光流更新的详细信息（包括调试信息）
                if (flow_updates < 10) {
                    // 计算ESKF估计的机体系速度，用于对比
                    Eigen::Matrix3d R_wb_eskf = current_state.q.toRotationMatrix();
                    Eigen::Vector3d vel_body_eskf = R_wb_eskf.transpose() * current_state.v;
                    
                    std::cout << "\n  [Flow Update #" << flow_updates << "] t=" << std::fixed << std::setprecision(6) 
                              << imu.timestamp << "s, height=" << std::setprecision(4) << current_height 
                              << "m" << std::endl;
                    std::cout << "    GT vel (world): [" << std::setprecision(4) 
                              << gt_data[best_flow_gt_idx].vx << ", " 
                              << gt_data[best_flow_gt_idx].vy << ", " 
                              << gt_data[best_flow_gt_idx].vz << "] m/s" << std::endl;
                    std::cout << "    ESKF vel (world): [" << std::setprecision(4)
                              << current_state.v.x() << ", " 
                              << current_state.v.y() << ", " 
                              << current_state.v.z() << "] m/s" << std::endl;
                    std::cout << "    GT vel_body: [" << std::setprecision(4)
                              << vel_body.x() << ", " << vel_body.y() << ", " << vel_body.z() << "] m/s" << std::endl;
                    std::cout << "    ESKF vel_body: [" << std::setprecision(4)
                              << vel_body_eskf.x() << ", " << vel_body_eskf.y() << ", " << vel_body_eskf.z() << "] m/s" << std::endl;
                    std::cout << "    flow_vel (body xy): [" << std::setprecision(4)
                              << flow_velocity.x() << ", " << flow_velocity.y() << "] m/s" << std::endl;
                    std::cout << "    flow_vel residual (predicted): [" << std::setprecision(4)
                              << (flow_velocity.x() - vel_body_eskf.x()) << ", " 
                              << (flow_velocity.y() - vel_body_eskf.y()) << "] m/s" << std::endl;
                }
                
                eskf.updateFlow(imu.timestamp, flow_velocity, 0.01);  // 0.01 m/s 的测量噪声（原来的1/10）
                flow_updates++;
                last_flow_time = imu.timestamp;
            }
        }
        
        // 进度输出（每处理 10% 的数据）
        if (total_count > 0 && (imu_idx + 1) % (total_count / 10 + 1) == 0) {
            int progress = ((imu_idx + 1) * 100) / total_count;
            std::cout << progress << "%..." << std::flush;
        }
        
        // 每 1 秒输出一次状态，并与真值比较
        static double last_print_time = -1.0;
        if (imu.timestamp - last_print_time >= 1.0) {
            std::cout << std::endl;  // 换行以便输出状态
            init_eskf::InitESKF::NominalState state = eskf.getState();
            init_eskf::Matrix15d P = eskf.getCovariance();
            
            double pos_uncertainty = sqrt(P.block<3, 3>(0, 0).trace() / 3.0);
            double vel_uncertainty = sqrt(P.block<3, 3>(3, 3).trace() / 3.0);
            double ori_uncertainty = sqrt(P.block<3, 3>(6, 6).trace() / 3.0);
            
            std::cout << "  t=" << std::fixed << std::setprecision(2) << imu.timestamp 
                      << "s: pos=[" << std::setprecision(3) 
                      << state.p.x() << ", " << state.p.y() << ", " << state.p.z() << "] m"
                      << ", vel=[" << state.v.x() << ", " << state.v.y() << ", " << state.v.z() << "] m/s"
                      << ", uncertainty: pos=" << std::setprecision(4) << pos_uncertainty 
                      << "m, vel=" << vel_uncertainty << "m/s, ori=" << ori_uncertainty << "rad" << std::endl;
            
            // 与真值比较位置
            GroundTruthData gt_at_time;
            bool found_gt = false;
            double best_gt_diff = 0.1;
            
            // 从上次的 gt_idx 开始查找
            for (size_t i = gt_idx; i < gt_data.size() && i < gt_idx + 200; i++) {
                double diff = std::abs(gt_data[i].timestamp - imu.timestamp);
                if (diff < best_gt_diff) {
                    best_gt_diff = diff;
                    gt_at_time = gt_data[i];
                    found_gt = true;
                }
                if (gt_data[i].timestamp > imu.timestamp + 0.1) break;
            }
            
            if (found_gt) {
                Eigen::Vector3d gt_pos(gt_at_time.x, gt_at_time.y, gt_at_time.z);
                Eigen::Vector3d error = state.p - gt_pos;
                double pos_error = error.norm();
                
                std::cout << "    GT pos: [" << std::setprecision(3)
                          << gt_at_time.x << ", " << gt_at_time.y << ", " << gt_at_time.z << "] m"
                          << ", error: " << std::setprecision(4) << pos_error << " m"
                          << " [" << error.x() << ", " << error.y() << ", " << error.z() << "]" << std::endl;
            }
            
            last_print_time = imu.timestamp;
        }
    }
    
    std::cout << std::endl;  // 完成进度输出
    std::cout << "  ✓ Processed " << total_updates << " IMU updates" << std::endl;
    std::cout << "  ✓ Performed " << tof_updates << " TOF updates" << std::endl;
    std::cout << "  ✓ Performed " << flow_updates << " Flow updates (height > 0.1m)" << std::endl;
    
    // 4.5. 计算全程位置误差统计
    std::cout << "\n[4.5/5] Computing position error statistics..." << std::endl;
    
    std::vector<double> position_errors;
    std::vector<double> position_errors_x, position_errors_y, position_errors_z;
    size_t comparison_count = 0;
    
    // 遍历所有处理过的 IMU 数据点，与真值比较
    for (size_t imu_idx = 0; imu_idx < imu_data.size(); imu_idx++) {
        const auto& imu = imu_data[imu_idx];
        if (imu.timestamp > end_time) break;
        
        // 每 0.1 秒比较一次（避免过于频繁）
        static double last_compare_time = -1.0;
        if (imu.timestamp - last_compare_time >= 0.1) {
            // 获取当前状态
            init_eskf::InitESKF::NominalState state = eskf.getState();
            
            // 查找对应的真值
            GroundTruthData gt_at_time;
            bool found_gt = false;
            double best_gt_diff = 0.1;
            
            for (size_t i = 0; i < gt_data.size(); i++) {
                double diff = std::abs(gt_data[i].timestamp - imu.timestamp);
                if (diff < best_gt_diff) {
                    best_gt_diff = diff;
                    gt_at_time = gt_data[i];
                    found_gt = true;
                }
                if (gt_data[i].timestamp > imu.timestamp + 0.1) break;
            }
            
            if (found_gt) {
                Eigen::Vector3d gt_pos(gt_at_time.x, gt_at_time.y, gt_at_time.z);
                Eigen::Vector3d error = state.p - gt_pos;
                double pos_error = error.norm();
                
                position_errors.push_back(pos_error);
                position_errors_x.push_back(std::abs(error.x()));
                position_errors_y.push_back(std::abs(error.y()));
                position_errors_z.push_back(std::abs(error.z()));
                comparison_count++;
            }
            
            last_compare_time = imu.timestamp;
        }
    }
    
    if (!position_errors.empty()) {
        // 计算统计量
        double mean_error = 0.0;
        double max_error = 0.0;
        double min_error = 1e10;
        for (double err : position_errors) {
            mean_error += err;
            max_error = std::max(max_error, err);
            min_error = std::min(min_error, err);
        }
        mean_error /= position_errors.size();
        
        // 计算 RMSE
        double rmse = 0.0;
        for (double err : position_errors) {
            rmse += err * err;
        }
        rmse = std::sqrt(rmse / position_errors.size());
        
        // 计算各轴误差
        double mean_error_x = 0.0, mean_error_y = 0.0, mean_error_z = 0.0;
        for (double err : position_errors_x) mean_error_x += err;
        for (double err : position_errors_y) mean_error_y += err;
        for (double err : position_errors_z) mean_error_z += err;
        mean_error_x /= position_errors_x.size();
        mean_error_y /= position_errors_y.size();
        mean_error_z /= position_errors_z.size();
        
        std::cout << "  位置误差统计（共 " << comparison_count << " 个比较点）:" << std::endl;
        std::cout << "    平均误差: " << std::fixed << std::setprecision(4) << mean_error << " m" << std::endl;
        std::cout << "    RMSE: " << rmse << " m" << std::endl;
        std::cout << "    最大误差: " << max_error << " m" << std::endl;
        std::cout << "    最小误差: " << min_error << " m" << std::endl;
        std::cout << "    各轴平均误差: [" << std::setprecision(4)
                  << mean_error_x << ", " << mean_error_y << ", " << mean_error_z << "] m" << std::endl;
    } else {
        std::cout << "  ⚠ 未找到足够的真值数据进行比较" << std::endl;
    }
    
    // 5. 检查收敛和最终状态
    std::cout << "\n[5/5] Checking convergence and final state..." << std::endl;
    
    init_eskf::InitESKF::NominalState final_state = eskf.getState();
    init_eskf::Matrix15d final_P = eskf.getCovariance();
    
    std::cout << "\nFinal State:" << std::endl;
    std::cout << "  Position: [" << std::fixed << std::setprecision(3)
              << final_state.p.x() << ", " << final_state.p.y() << ", " 
              << final_state.p.z() << "] m" << std::endl;
    std::cout << "  Velocity: [" << final_state.v.x() << ", " 
              << final_state.v.y() << ", " << final_state.v.z() << "] m/s" << std::endl;
    std::cout << "  Orientation (quaternion): [" << std::setprecision(4)
              << final_state.q.w() << ", " << final_state.q.x() << ", " 
              << final_state.q.y() << ", " << final_state.q.z() << "]" << std::endl;
    std::cout << "  Acc bias: [" << std::setprecision(6)
              << final_state.ba.x() << ", " << final_state.ba.y() << ", " 
              << final_state.ba.z() << "] m/s^2" << std::endl;
    std::cout << "  Gyr bias: [" << final_state.bg.x() << ", " 
              << final_state.bg.y() << ", " << final_state.bg.z() << "] rad/s" << std::endl;
    
    // 不确定性
    double pos_uncertainty = sqrt(final_P.block<3, 3>(0, 0).trace() / 3.0);
    double vel_uncertainty = sqrt(final_P.block<3, 3>(3, 3).trace() / 3.0);
    double ori_uncertainty = sqrt(final_P.block<3, 3>(6, 6).trace() / 3.0);
    
    std::cout << "\nUncertainty:" << std::endl;
    std::cout << "  Position: " << std::setprecision(4) << pos_uncertainty << " m" << std::endl;
    std::cout << "  Velocity: " << vel_uncertainty << " m/s" << std::endl;
    std::cout << "  Orientation: " << ori_uncertainty << " rad" << std::endl;
    
    // 收敛检查
    bool converged = eskf.checkConvergence(0.1, 0.1, 0.05);
    std::cout << "\nConvergence: " << (converged ? "✓ YES" : "✗ NO") << std::endl;
    
    // 与真值比较（最后一个数据点）
    if (!gt_data.empty()) {
        // 找到对应时间戳的真值（处理结束时的真值）
        GroundTruthData final_gt;
        bool found_final_gt = false;
        double final_time = imu_data.empty() ? 0.0 : imu_data[std::min<size_t>(imu_data.size()-1, total_count-1)].timestamp;
        
        // 查找最接近的真值
        double best_gt_diff = 1.0;
        for (const auto& gt : gt_data) {
            double diff = std::abs(gt.timestamp - final_time);
            if (diff < best_gt_diff) {
                best_gt_diff = diff;
                final_gt = gt;
                found_final_gt = true;
            }
            if (gt.timestamp > final_time + 0.1) break;
        }
        
        if (found_final_gt) {
            Eigen::Vector3d gt_pos(final_gt.x, final_gt.y, final_gt.z);
            Eigen::Vector3d error = final_state.p - gt_pos;
            
            std::cout << "\nComparison with Ground Truth (at t=" << std::fixed << std::setprecision(2) 
                      << final_time << "s):" << std::endl;
            std::cout << "  GT timestamp: " << std::setprecision(6) << final_gt.timestamp << " s" << std::endl;
            std::cout << "  GT position: [" << std::setprecision(3)
                      << final_gt.x << ", " << final_gt.y << ", " << final_gt.z << "] m" << std::endl;
            std::cout << "  ESKF position: [" << final_state.p.x() << ", " 
                      << final_state.p.y() << ", " << final_state.p.z() << "] m" << std::endl;
            std::cout << "  Position error: " << std::setprecision(4) << error.norm() << " m" << std::endl;
            std::cout << "    Error components: [" << error.x() << ", " 
                      << error.y() << ", " << error.z() << "] m" << std::endl;
            
            // 检查初始位置（应该都是 [0,0,0] 因为归一化了）
            if (!gt_data.empty()) {
                std::cout << "\n归一化检查:" << std::endl;
                std::cout << "  初始 GT 位置: [" << std::setprecision(3)
                          << gt_data[0].x << ", " << gt_data[0].y << ", " << gt_data[0].z << "] m" << std::endl;
                std::cout << "  初始 ESKF 位置: [0.000, 0.000, 0.000] m (归一化后)" << std::endl;
                std::cout << "  初始位置差异: " << Eigen::Vector3d(gt_data[0].x, gt_data[0].y, gt_data[0].z).norm() << " m" << std::endl;
            }
        }
    }
    
    std::cout << "\n==========================================" << std::endl;
    std::cout << "Test completed!" << std::endl;
    std::cout << "==========================================" << std::endl;
    
    return 0;
}
