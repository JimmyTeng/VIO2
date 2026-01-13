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

#ifndef OV_INIT_DYNAMICINITIALIZER_H
#define OV_INIT_DYNAMICINITIALIZER_H

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
class PoseJPL;
class Landmark;
class Vec;
} // namespace ov_type

namespace ov_init {

/**
 * @brief 动态视觉惯性系统的初始化器。
 *
 * 此实现将尝试恢复系统的初始条件。
 * 此外，我们将尝试恢复系统的协方差。
 * 使用任意运动进行初始化：
 * 1. 预积分系统以获得相对旋转变化（假设偏差已知）
 * 2. 构建带特征的线性系统以恢复速度（使用 |g| 约束求解）
 * 3. 对所有校准执行大型 MLE 并恢复协方差。
 *
 * 方法基于以下工作（有关高级概述，请参阅此 [技术报告](https://pgeneva.com/downloads/reports/tr_init.pdf)）：
 *
 * > Dong-Si, Tue-Cuong, and Anastasios I. Mourikis.
 * > "Estimator initialization in vision-aided inertial navigation with unknown camera-IMU calibration."
 * > 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.
 *
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 *
 */
class DynamicInitializer {
public:
  /**
   * @brief 默认构造函数
   * @param params_ 从 ROS 或 CMDLINE 加载的参数
   * @param db 包含所有特征的特征跟踪器数据库
   * @param imu_data_ 指向我们的 IMU 历史信息向量的共享指针
   */
  explicit DynamicInitializer(const InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db,
                              std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief 尝试获取初始化的系统
   *
   * @param[out] timestamp 我们初始化状态的时间戳（最后一个 IMU 状态）
   * @param[out] covariance 返回状态的已计算协方差
   * @param[out] order 协方差矩阵的顺序
   * @param _imu 指向"活动" IMU 状态的指针 (q_GtoI, p_IinG, v_IinG, bg, ba)
   * @param _clones_IMU 成像时间与克隆姿态之间的映射 (q_GtoIi, p_IiinG)
   * @param _features_SLAM 我们当前的 SLAM 特征集（3D 位置）
   * @return 如果我们成功初始化了系统，则返回 True
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> &_imu, std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
                  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &_features_SLAM);

private:
  /// 初始化参数
  InertialInitializerOptions params;

  /// 包含所有特征的特征跟踪器数据库
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// 我们的 IMU 消息历史（时间、角速度、线加速度）
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;
};

} // namespace ov_init

#endif // OV_INIT_DYNAMICINITIALIZER_H
