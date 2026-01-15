/*******************************************************
 * EuRoC 数据集测试脚本
 *
 * 测试 Init-ESKF 在 V1_01_easy 数据集上的表现
 * 初始点归一化为 0
 *******************************************************/

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "init_eskf/euroc_parser.h"
#include "init_eskf/init_eskf.h"

int main(int argc, char** argv) {
  // 数据集路径
  std::string dataset_path =
      "/home/jimmy/project/vio_ws/src/data/EuRoC/V1_02_medium";

  if (argc > 1) {
    dataset_path = argv[1];
  }
  init_eskf::InitESKF eskf;
  eskf.initialize(9.81, 0.01, 0.001, 1e-5, 1e-6);
  init_eskf::EuroCParser parser;
  parser.setDatasetPath(dataset_path);

  int64_t count = 0;
  int64_t first_image_timestamp = 0;
  int64_t first_imu_timestamp = 0;
  int64_t first_ground_truth_timestamp = 0;
  const int64_t duration_ns = 10LL * 1000000000LL;  // 10秒，单位纳秒

  // 统计信息
  int imu_count = 0;
  int zupt_count = 0;
  int propagate_count = 0;
  int64_t last_print_time = 0;
  const int64_t print_interval_ns = 100000000LL;  // 每0.1秒打印一次
  
  // 存储最近的真值数据用于对比
  init_eskf::GroundTruthData latest_gt;
  bool has_gt = false;
  
  // 第一个真值数据点，用于坐标系转换
  init_eskf::GroundTruthData first_gt;
  bool has_first_gt = false;
  Eigen::Vector3d first_gt_position;
  Eigen::Quaterniond first_gt_orientation;
  
  // 调试：打印前几帧原始IMU数据
  bool debug_imu_printed = false;
  const int debug_imu_frames = 5;

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "========================================\n";
  std::cout << "开始处理数据（仅前10秒）\n";
  std::cout << "========================================\n";

  while (true) {
    auto data = parser.popNext();
    if (!data) {
      break;
    }



    // 检查是否超过10秒
    if (first_imu_timestamp > 0) {
      int64_t elapsed_ns = data->getTimestamp() - first_imu_timestamp;
      if (elapsed_ns > duration_ns) {
        std::cout << "\n已达到10秒限制，停止处理数据\n";
        break;
      }
    }

    if (data->type == init_eskf::DataType::IMAGE) {
      if (first_image_timestamp == 0) {
        first_image_timestamp = data->getTimestamp();
      }
    } else if (data->type == init_eskf::DataType::IMU) {
      if (first_imu_timestamp == 0) {
        first_imu_timestamp = data->getTimestamp();
        std::cout << "开始时间戳: " << first_imu_timestamp << " ns\n";
      }
    } else if (data->type == init_eskf::DataType::GROUND_TRUTH) {
      if (first_ground_truth_timestamp == 0) {
        first_ground_truth_timestamp = data->getTimestamp();
      }
    }

    if (first_imu_timestamp != 0 && first_ground_truth_timestamp != 0) {
      if (data->type == init_eskf::DataType::IMU) {
        imu_count++;
        
        // 调试：打印前几帧转换后的IMU数据（已转换为FLU坐标系）
        if (!debug_imu_printed && imu_count <= debug_imu_frames) {
          double acc_norm = data->getIMU().acc.norm();
          std::cout << "[调试] IMU数据 #" << imu_count << " (FLU坐标系): "
                    << "acc=[" << data->getIMU().acc(0) << ", " 
                    << data->getIMU().acc(1) << ", " 
                    << data->getIMU().acc(2) << "] m/s², "
                    << "模长=" << acc_norm << " m/s², "
                    << "gyr=[" << data->getIMU().gyr(0) << ", " 
                    << data->getIMU().gyr(1) << ", " 
                    << data->getIMU().gyr(2) << "] rad/s\n";
          if (imu_count == debug_imu_frames) {
            debug_imu_printed = true;
            std::cout << "[调试] 前" << debug_imu_frames 
                      << "帧IMU数据打印完成（已转换为FLU坐标系）\n";
            std::cout << "[调试] 注意：静止平放时，加速度应接近 [0, 0, 9.81] m/s²\n";
          }
        }
        
        if (eskf.getGravityAlignmentStatus() !=
            init_eskf::GravityAlignmentStatus::COMPLETED) {
          // 使用逐个添加数据的方式，自动开始和完成初始化
          bool alignment_result = eskf.addGravityAlignmentData(
              data->getTimestamp(), data->getIMU().acc, data->getIMU().gyr);

          // 检查重力对齐是否完成或失败
          auto status = eskf.getGravityAlignmentStatus();

          // 如果 addGravityAlignmentData 返回 true 但状态不是 COMPLETED，说明对齐失败
          if (alignment_result &&
              status != init_eskf::GravityAlignmentStatus::COMPLETED) {
            std::cout << "[警告] 重力对齐失败！可能原因：系统在运动或数据不足\n";
            std::cout << "       当前状态: ";
            if (status == init_eskf::GravityAlignmentStatus::NOT_STARTED) {
              std::cout << "NOT_STARTED\n";
            } else if (status == init_eskf::GravityAlignmentStatus::IN_PROGRESS) {
              std::cout << "IN_PROGRESS\n";
            } else if (status == init_eskf::GravityAlignmentStatus::FAILED) {
              std::cout << "FAILED\n";
            }
          }

          // 打印重力对齐状态
          if (imu_count % 100 == 0) {
            std::string status_str = "NOT_STARTED";
            if (status == init_eskf::GravityAlignmentStatus::IN_PROGRESS) {
              status_str = "IN_PROGRESS";
            } else if (status == init_eskf::GravityAlignmentStatus::COMPLETED) {
              status_str = "COMPLETED";
            } else if (status == init_eskf::GravityAlignmentStatus::FAILED) {
              status_str = "FAILED";
            }
            std::cout << "[初始化中] 重力对齐状态: " << status_str << "\n";
          }

          // 如果对齐成功，打印成功信息
          if (status == init_eskf::GravityAlignmentStatus::COMPLETED) {
            std::cout << "[成功] 重力对齐完成！\n";
          }

          // 如果对齐失败，打印失败信息
          if (status == init_eskf::GravityAlignmentStatus::FAILED) {
            std::cout << "[失败] 重力对齐失败！\n";
          }
        } else {

// 1. 无论状态如何，总是先进行传播（积分 IMU 数据，更新协方差）
          eskf.propagate(data->getTimestamp(), data->getIMU().acc, data->getIMU().gyr);
          propagate_count++;

          // 2. 判断是否满足零速更新条件
          bool is_zupt = eskf.detectStaticState(data->getIMU().acc, data->getIMU().gyr, 0.5, 0.1);

          if (is_zupt) {
            // 3. 如果静止，执行观测更新
            eskf.updateZUPT(data->getTimestamp());
            zupt_count++;
          }

          // 定期打印滤波器状态
          if (first_imu_timestamp > 0 &&
              (data->getTimestamp() - last_print_time) >= print_interval_ns) {
            last_print_time = data->getTimestamp();
            double elapsed_sec = (data->getTimestamp() - first_imu_timestamp) / 1e9;

            auto state = eskf.getState();
            auto cov = eskf.getCovariance();

            // 将四元数转换为欧拉角（roll, pitch, yaw）
            Eigen::Vector3d euler = state.q.toRotationMatrix().eulerAngles(2, 1, 0);
            double roll = euler(2);
            double pitch = euler(1);
            double yaw = euler(0);

            // 计算位置、速度、姿态的不确定性（标准差）
            double pos_std = sqrt(cov.block<3, 3>(0, 0).trace() / 3.0);
            double vel_std = sqrt(cov.block<3, 3>(3, 3).trace() / 3.0);
            double att_std = sqrt(cov.block<3, 3>(6, 6).trace() / 3.0);

            std::cout << "\n----------------------------------------\n";
            std::cout << "时间: " << elapsed_sec << " s\n";
            std::cout << "位置 (m): [" << state.p(0) << ", " << state.p(1)
                      << ", " << state.p(2) << "]\n";
            std::cout << "速度 (m/s): [" << state.v(0) << ", " << state.v(1)
                      << ", " << state.v(2) << "]\n";
            std::cout << "姿态 (rad): roll=" << roll << ", pitch=" << pitch
                      << ", yaw=" << yaw << "\n";
            std::cout << "姿态 (deg): roll=" << roll * 180.0 / M_PI
                      << ", pitch=" << pitch * 180.0 / M_PI
                      << ", yaw=" << yaw * 180.0 / M_PI << "\n";
            std::cout << "加速度计偏置 (m/s²): [" << state.ba(0) << ", "
                      << state.ba(1) << ", " << state.ba(2) << "]\n";
            std::cout << "陀螺仪偏置 (rad/s): [" << state.bg(0) << ", "
                      << state.bg(1) << ", " << state.bg(2) << "]\n";
            std::cout << "位置不确定性 (m): " << pos_std << "\n";
            std::cout << "速度不确定性 (m/s): " << vel_std << "\n";
            std::cout << "姿态不确定性 (rad): " << att_std << "\n";
            
            // 与真值对比
            if (has_gt) {
              // 计算时间差，如果真值数据太旧（超过0.1秒），不进行对比
              int64_t time_diff_ns = std::abs(data->getTimestamp() - latest_gt.timestamp);
              double time_diff_sec = time_diff_ns / 1e9;
              
              if (time_diff_sec < 0.1) {  // 真值数据在0.1秒内有效
                // 位置误差
                Eigen::Vector3d pos_error(
                    state.p(0) - latest_gt.x,
                    state.p(1) - latest_gt.y,
                    state.p(2) - latest_gt.z);
                double pos_error_norm = pos_error.norm();
                
                // 速度误差
                Eigen::Vector3d vel_error(
                    state.v(0) - latest_gt.vx,
                    state.v(1) - latest_gt.vy,
                    state.v(2) - latest_gt.vz);
                double vel_error_norm = vel_error.norm();
                
                // 姿态误差（四元数）
                Eigen::Quaterniond gt_q(latest_gt.qw, latest_gt.qx, latest_gt.qy, latest_gt.qz);
                Eigen::Quaterniond q_error = state.q * gt_q.inverse();
                // 将四元数误差转换为角度误差（度）
                double angle_error_rad = 2.0 * acos(std::abs(q_error.w()));
                double angle_error_deg = angle_error_rad * 180.0 / M_PI;
                
                // 真值姿态的欧拉角
                Eigen::Vector3d gt_euler = gt_q.toRotationMatrix().eulerAngles(2, 1, 0);
                
                // ESKF姿态的欧拉角
                Eigen::Vector3d eskf_euler = state.q.toRotationMatrix().eulerAngles(2, 1, 0);
                
                // 计算各轴的角度误差（分别计算roll, pitch, yaw误差）
                Eigen::Vector3d error_euler = eskf_euler - gt_euler;
                // 将角度误差归一化到[-180, 180]度
                for (int i = 0; i < 3; i++) {
                  while (error_euler(i) > M_PI) error_euler(i) -= 2 * M_PI;
                  while (error_euler(i) < -M_PI) error_euler(i) += 2 * M_PI;
                }
                
                std::cout << "\n--- 与真值对比（已转换到0点坐标系）---\n";
                std::cout << "真值位置 (m): [" << latest_gt.x << ", " 
                          << latest_gt.y << ", " << latest_gt.z << "]\n";
                std::cout << "位置误差 (m): [" << pos_error(0) << ", " 
                          << pos_error(1) << ", " << pos_error(2) << "]\n";
                std::cout << "位置误差模长 (m): " << pos_error_norm << "\n";
                
                std::cout << "真值速度 (m/s): [" << latest_gt.vx << ", " 
                          << latest_gt.vy << ", " << latest_gt.vz << "]\n";
                std::cout << "速度误差 (m/s): [" << vel_error(0) << ", " 
                          << vel_error(1) << ", " << vel_error(2) << "]\n";
                std::cout << "速度误差模长 (m/s): " << vel_error_norm << "\n";
                
                std::cout << "ESKF姿态 (deg): roll=" << eskf_euler(2) * 180.0 / M_PI 
                          << ", pitch=" << eskf_euler(1) * 180.0 / M_PI 
                          << ", yaw=" << eskf_euler(0) * 180.0 / M_PI << "\n";
                std::cout << "真值姿态 (deg): roll=" << gt_euler(2) * 180.0 / M_PI 
                          << ", pitch=" << gt_euler(1) * 180.0 / M_PI 
                          << ", yaw=" << gt_euler(0) * 180.0 / M_PI << "\n";
                std::cout << "姿态误差分量 (deg): roll=" << error_euler(2) * 180.0 / M_PI 
                          << ", pitch=" << error_euler(1) * 180.0 / M_PI 
                          << ", yaw=" << error_euler(0) * 180.0 / M_PI << "\n";
                std::cout << "姿态总误差 (deg): " << angle_error_deg << "\n";
                std::cout << "时间差: " << time_diff_sec * 1000.0 << " ms\n";
              } else {
                std::cout << "\n--- 真值数据过期 (时间差: " 
                          << time_diff_sec * 1000.0 << " ms) ---\n";
              }
            } else {
              std::cout << "\n--- 暂无真值数据用于对比 ---\n";
            }
            
            std::cout << "统计: IMU=" << imu_count << ", Propagate="
                      << propagate_count << ", ZUPT=" << zupt_count << "\n";
            std::cout << "----------------------------------------\n";
          }
        }
      } else if (data->type == init_eskf::DataType::GROUND_TRUTH) {
        // 记录第一个真值数据点作为参考坐标系
        // 注意：真值数据使用FLU坐标系（World坐标系），与IMU数据（Body坐标系，已转换为FLU）一致
        if (!has_first_gt) {
          first_gt = data->getGroundTruth();
          first_gt_position = Eigen::Vector3d(first_gt.x, first_gt.y, first_gt.z);
          first_gt_orientation = Eigen::Quaterniond(
              first_gt.qw, first_gt.qx, first_gt.qy, first_gt.qz);
          has_first_gt = true;
          
          // 打印第一个真值点的姿态信息
          Eigen::Vector3d first_euler = first_gt_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
          std::cout << "\n[真值坐标系] 第一个真值点（原始，FLU坐标系）: 位置=[" 
                    << first_gt.x << ", " << first_gt.y << ", " << first_gt.z << "]\n";
          std::cout << "[真值坐标系] 第一个真值点姿态 (deg): roll=" 
                    << first_euler(2) * 180.0 / M_PI 
                    << ", pitch=" << first_euler(1) * 180.0 / M_PI 
                    << ", yaw=" << first_euler(0) * 180.0 / M_PI << "\n";
          std::cout << "[真值坐标系] 注意：只归一化位置到(0,0,0)，姿态保持绝对姿态\n";
          std::cout << "[真值坐标系] 原因：真值姿态已经和重力对齐，不应强制归一化\n";
          std::cout << "[坐标系确认] 真值数据：FLU坐标系（World），IMU数据：FLU坐标系（Body，已从URF转换）\n";
        }
        
        // 将真值转换到0点坐标系
        init_eskf::GroundTruthData gt_normalized = data->getGroundTruth();
        
        // 位置转换：减去第一个位置（归一化位置）
        gt_normalized.x -= first_gt_position(0);
        gt_normalized.y -= first_gt_position(1);
        gt_normalized.z -= first_gt_position(2);
        
        // 姿态转换：保持绝对姿态，不归一化
        // 原因：真值姿态已经和重力对齐（Vicon坐标系），如果归一化会引入人为误差
        // 只旋转速度向量到新的位置坐标系（如果需要）
        // 注意：姿态保持原样，不进行转换
        // gt_normalized的四元数保持不变
        
        // 速度：在真值坐标系中，速度是相对于世界坐标系的，不需要转换
        // 但如果位置归一化了，速度也应该在同一个坐标系下
        // 由于我们只归一化了位置，速度保持不变（已经是世界坐标系下的速度）
        
        // 保存转换后的真值数据（位置归一化，姿态保持绝对）
        latest_gt = gt_normalized;
        has_gt = true;
        
        // updateTOF 使用转换后的z坐标（位置已归一化）
        // 注意：真值的z坐标对应高度，应该使用z而不是x
        eskf.updateTOF(data->getTimestamp(), gt_normalized.z, 0.05);
      }
    }
    count++;
  }

  // 打印最终结果
  std::cout << "\n========================================\n";
  std::cout << "处理完成\n";
  std::cout << "========================================\n";
  std::cout << "first_image_timestamp: " << first_image_timestamp << "\n";
  std::cout << "first_imu_timestamp: " << first_imu_timestamp << "\n";
  std::cout << "first_ground_truth_timestamp: " << first_ground_truth_timestamp
            << "\n";
  std::cout << "总数据量: " << count << "\n";
  std::cout << "IMU数据量: " << imu_count << "\n";
  std::cout << "Propagate次数: " << propagate_count << "\n";
  std::cout << "ZUPT次数: " << zupt_count << "\n";

  // 检查重力对齐最终状态
  auto final_status = eskf.getGravityAlignmentStatus();
  std::cout << "\n重力对齐最终状态: ";
  if (final_status == init_eskf::GravityAlignmentStatus::NOT_STARTED) {
    std::cout << "NOT_STARTED (未开始)\n";
  } else if (final_status == init_eskf::GravityAlignmentStatus::IN_PROGRESS) {
    std::cout << "IN_PROGRESS (进行中，10秒内未完成)\n";
    std::cout << "[警告] 重力对齐未能在10秒内完成！\n";
  } else if (final_status == init_eskf::GravityAlignmentStatus::COMPLETED) {
    std::cout << "COMPLETED (成功)\n";
  } else if (final_status == init_eskf::GravityAlignmentStatus::FAILED) {
    std::cout << "FAILED (失败)\n";
    std::cout << "[错误] 重力对齐失败！可能原因：系统在运动、数据不足或加速度方差过大\n";
  }

  // 打印最终状态
  if (final_status == init_eskf::GravityAlignmentStatus::COMPLETED) {
    auto state = eskf.getState();
    auto cov = eskf.getCovariance();
    Eigen::Vector3d euler = state.q.toRotationMatrix().eulerAngles(2, 1, 0);

    // 计算不确定性
    double pos_std = sqrt(cov.block<3, 3>(0, 0).trace() / 3.0);
    double vel_std = sqrt(cov.block<3, 3>(3, 3).trace() / 3.0);
    double att_std = sqrt(cov.block<3, 3>(6, 6).trace() / 3.0);

    std::cout << "\n最终状态:\n";
    std::cout << "位置 (m): [" << state.p(0) << ", " << state.p(1)
              << ", " << state.p(2) << "]\n";
    std::cout << "速度 (m/s): [" << state.v(0) << ", " << state.v(1)
              << ", " << state.v(2) << "]\n";
    std::cout << "姿态 (deg): roll=" << euler(2) * 180.0 / M_PI
              << ", pitch=" << euler(1) * 180.0 / M_PI
              << ", yaw=" << euler(0) * 180.0 / M_PI << "\n";
    std::cout << "加速度计偏置 (m/s²): [" << state.ba(0) << ", "
              << state.ba(1) << ", " << state.ba(2) << "]\n";
    std::cout << "陀螺仪偏置 (rad/s): [" << state.bg(0) << ", "
              << state.bg(1) << ", " << state.bg(2) << "]\n";
    std::cout << "位置不确定性 (m): " << pos_std << "\n";
    std::cout << "速度不确定性 (m/s): " << vel_std << "\n";
    std::cout << "姿态不确定性 (rad): " << att_std << "\n";
    
    // 最终状态与真值对比
    if (has_gt) {
      // 位置误差
      Eigen::Vector3d pos_error(
          state.p(0) - latest_gt.x,
          state.p(1) - latest_gt.y,
          state.p(2) - latest_gt.z);
      double pos_error_norm = pos_error.norm();
      
      // 速度误差
      Eigen::Vector3d vel_error(
          state.v(0) - latest_gt.vx,
          state.v(1) - latest_gt.vy,
          state.v(2) - latest_gt.vz);
      double vel_error_norm = vel_error.norm();
      
      // 姿态误差
      Eigen::Quaterniond gt_q(latest_gt.qw, latest_gt.qx, latest_gt.qy, latest_gt.qz);
      Eigen::Quaterniond q_error = state.q * gt_q.inverse();
      double angle_error_rad = 2.0 * acos(std::abs(q_error.w()));
      double angle_error_deg = angle_error_rad * 180.0 / M_PI;
      
      Eigen::Vector3d gt_euler = gt_q.toRotationMatrix().eulerAngles(2, 1, 0);
      Eigen::Vector3d eskf_euler = state.q.toRotationMatrix().eulerAngles(2, 1, 0);
      
      // 计算各轴的角度误差
      Eigen::Vector3d error_euler = eskf_euler - gt_euler;
      for (int i = 0; i < 3; i++) {
        while (error_euler(i) > M_PI) error_euler(i) -= 2 * M_PI;
        while (error_euler(i) < -M_PI) error_euler(i) += 2 * M_PI;
      }
      
      std::cout << "\n--- 最终状态与真值对比（已转换到0点坐标系）---\n";
      std::cout << "真值位置 (m): [" << latest_gt.x << ", " 
                << latest_gt.y << ", " << latest_gt.z << "]\n";
      std::cout << "位置误差 (m): [" << pos_error(0) << ", " 
                << pos_error(1) << ", " << pos_error(2) << "]\n";
      std::cout << "位置误差模长 (m): " << pos_error_norm << "\n";
      
      std::cout << "真值速度 (m/s): [" << latest_gt.vx << ", " 
                << latest_gt.vy << ", " << latest_gt.vz << "]\n";
      std::cout << "速度误差 (m/s): [" << vel_error(0) << ", " 
                << vel_error(1) << ", " << vel_error(2) << "]\n";
      std::cout << "速度误差模长 (m/s): " << vel_error_norm << "\n";
      
      std::cout << "ESKF姿态 (deg): roll=" << eskf_euler(2) * 180.0 / M_PI 
                << ", pitch=" << eskf_euler(1) * 180.0 / M_PI 
                << ", yaw=" << eskf_euler(0) * 180.0 / M_PI << "\n";
      std::cout << "真值姿态 (deg): roll=" << gt_euler(2) * 180.0 / M_PI 
                << ", pitch=" << gt_euler(1) * 180.0 / M_PI 
                << ", yaw=" << gt_euler(0) * 180.0 / M_PI << "\n";
      std::cout << "姿态误差分量 (deg): roll=" << error_euler(2) * 180.0 / M_PI 
                << ", pitch=" << error_euler(1) * 180.0 / M_PI 
                << ", yaw=" << error_euler(0) * 180.0 / M_PI << "\n";
      std::cout << "姿态总误差 (deg): " << angle_error_deg << "\n";
      
      // 分析角度误差原因
      std::cout << "\n--- 角度误差分析 ---\n";
      if (std::abs(error_euler(2)) * 180.0 / M_PI > 5.0) {
        std::cout << "[警告] Roll误差较大: " << error_euler(2) * 180.0 / M_PI 
                  << "度（重力对齐应该能准确估计roll）\n";
      }
      if (std::abs(error_euler(1)) * 180.0 / M_PI > 5.0) {
        std::cout << "[警告] Pitch误差较大: " << error_euler(1) * 180.0 / M_PI 
                  << "度（重力对齐应该能准确估计pitch）\n";
      }
      if (std::abs(error_euler(0)) * 180.0 / M_PI > 10.0) {
        std::cout << "[注意] Yaw误差较大: " << error_euler(0) * 180.0 / M_PI 
                  << "度（这是正常的，重力对齐无法确定yaw角）\n";
        std::cout << "       原因：重力对齐只对齐了重力方向（roll和pitch），\n";
        std::cout << "             无法确定绕重力轴的旋转（yaw角）\n";
        std::cout << "       解决方案：需要磁力计或其他传感器来确定yaw角\n";
      }
    }
  } else {
    // 重力对齐失败，仍然打印当前状态（可能不准确）
    std::cout << "\n[警告] 重力对齐未完成，以下状态可能不准确：\n";
    auto state = eskf.getState();
    auto cov = eskf.getCovariance();
    Eigen::Vector3d euler = state.q.toRotationMatrix().eulerAngles(2, 1, 0);

    double pos_std = sqrt(cov.block<3, 3>(0, 0).trace() / 3.0);
    double vel_std = sqrt(cov.block<3, 3>(3, 3).trace() / 3.0);
    double att_std = sqrt(cov.block<3, 3>(6, 6).trace() / 3.0);

    std::cout << "位置 (m): [" << state.p(0) << ", " << state.p(1)
              << ", " << state.p(2) << "]\n";
    std::cout << "速度 (m/s): [" << state.v(0) << ", " << state.v(1)
              << ", " << state.v(2) << "]\n";
    std::cout << "姿态 (deg): roll=" << euler(2) * 180.0 / M_PI
              << ", pitch=" << euler(1) * 180.0 / M_PI
              << ", yaw=" << euler(0) * 180.0 / M_PI << "\n";
    std::cout << "位置不确定性 (m): " << pos_std << "\n";
    std::cout << "速度不确定性 (m/s): " << vel_std << "\n";
    std::cout << "姿态不确定性 (rad): " << att_std << "\n";
    std::cout << "\n建议：确保系统在静止状态下开始，或增加重力对齐时间窗口\n";
  }

  return 0;
}
