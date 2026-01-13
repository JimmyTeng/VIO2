/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_INIT_SIMULATORINIT_H
#define OV_INIT_SIMULATORINIT_H

#include <Eigen/Eigen>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class BsplineSE3;
} // namespace ov_core

namespace ov_init {

/**
 * @brief 生成视觉惯性测量的主模拟器类
 *
 * 给定轨迹，这将为该轨迹生成 SE(3) @ref ov_core::BsplineSE3。
 * 这允许我们在此轨迹的每个时间步获取惯性测量信息。
 * 创建 B 样条后，我们将生成环境特征图，该图将用作我们的特征测量值。
 * 该地图将在每个时间步投影到帧中以获得我们的"原始" uv 测量值。
 * 我们将偏差和白噪声注入到惯性读数中，同时将白噪声添加到 uv 测量值中。
 * 用户应指定所需的传感器速率以及随机数生成器的种子。
 *
 */
class SimulatorInit {

public:
  /**
   * @brief 默认构造函数，将加载所有配置变量
   * @param params_ InertialInitializer 参数。应该已经从 cmd 加载。
   */
  SimulatorInit(InertialInitializerOptions &params_);

  /**
   * @brief 将获取一组扰动参数
   * @param params_ 我们将扰动的参数
   */
  void perturb_parameters(InertialInitializerOptions &params_);

  /**
   * @brief 返回我们是否正在主动模拟
   * @return 如果我们仍有模拟数据，则返回 True
   */
  bool ok() { return is_running; }

  /**
   * @brief 获取我们已经模拟到的时间戳
   * @return 时间戳
   */
  double current_timestamp() { return timestamp; }

  /**
   * @brief 获取指定时间步的模拟状态
   * @param desired_time 我们要获取状态的时间戳
   * @param imustate MSCKF 排序中的状态：[time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   * @return 如果我们有状态，则返回 True
   */
  bool get_state(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

  /**
   * @brief 获取下一个惯性读数（如果有）。
   * @param time_imu 此测量发生的时间
   * @param wm 惯性系中的角速度测量值
   * @param am 惯性系中的线加速度测量值
   * @return 如果我们有测量值，则返回 True
   */
  bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  /**
   * @brief 获取下一个相机读数（如果有）。
   * @param time_cam 此测量发生的时间
   * @param camids 对应向量匹配的相机 ID
   * @param feats 返回时间的带噪声 uv 测量值和 ID
   * @return 如果我们有测量值，则返回 True
   */
  bool get_next_cam(double &time_cam, std::vector<int> &camids, std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /// 返回特征的真实 3D 地图
  std::unordered_map<size_t, Eigen::Vector3d> get_map() { return featmap; }

  /// 访问函数以获取真实参数（即校准和设置）
  InertialInitializerOptions get_true_parameters() { return params; }

protected:
  /**
   * @brief 将传递的地图特征投影到所需的相机帧中。
   * @param R_GtoI IMU 姿态的方向
   * @param p_IinG IMU 姿态的位置
   * @param camid 我们要投影到的相机传感器的相机 ID
   * @param feats 我们的 3D 特征集
   * @return 指定相机的真实畸变原始图像测量值及其 ID
   */
  std::vector<std::pair<size_t, Eigen::VectorXf>> project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                                                                     int camid, const std::unordered_map<size_t, Eigen::Vector3d> &feats);

  /**
   * @brief 将在指定相机的视场中生成点
   * @param R_GtoI IMU 姿态的方向
   * @param p_IinG IMU 姿态的位置
   * @param camid 我们要投影到的相机传感器的相机 ID
   * @param[out] feats 我们将追加新特征的地图
   * @param numpts 我们应该生成的点数
   */
  void generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, int camid,
                       std::unordered_map<size_t, Eigen::Vector3d> &feats, int numpts);

  //===================================================================
  // 配置变量
  //===================================================================

  /// 真实参数（已解析参数的副本）
  InertialInitializerOptions params;

  //===================================================================
  // 状态相关变量
  //===================================================================

  /// 我们加载的轨迹数据（时间戳(秒), q_GtoI, p_IinG）
  std::vector<Eigen::VectorXd> traj_data;

  /// 我们的 B 样条轨迹
  std::shared_ptr<ov_core::BsplineSE3> spline;

  /// 我们的 3D 特征地图
  size_t id_map = 0;
  std::unordered_map<size_t, Eigen::Vector3d> featmap;

  /// 用于测量值的梅森旋转器伪随机数生成器（IMU）
  std::mt19937 gen_meas_imu;

  /// 用于测量值的梅森旋转器伪随机数生成器（相机）
  std::vector<std::mt19937> gen_meas_cams;

  /// 用于状态初始化的梅森旋转器伪随机数生成器
  std::mt19937 gen_state_init;

  /// 用于状态扰动的梅森旋转器伪随机数生成器
  std::mt19937 gen_state_perturb;

  /// 我们的模拟是否正在运行
  bool is_running;

  //===================================================================
  // 模拟特定变量
  //===================================================================

  /// 系统的当前时间戳
  double timestamp;

  /// 我们上次有 IMU 读数的时间
  double timestamp_last_imu;

  /// 我们上次有相机读数的时间
  double timestamp_last_cam;

  /// 我们的运行加速度偏差
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

  /// 我们的运行陀螺仪偏差
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // 我们的真实偏差历史
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;
};

} // namespace ov_init

#endif // OV_INIT_SIMULATORINIT_H
