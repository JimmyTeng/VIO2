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

#ifndef OV_INIT_CERES_IMUCPIV1_H
#define OV_INIT_CERES_IMUCPIV1_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief IMU 连续预积分版本 1 的因子
 */
class Factor_ImuCPIv1 : public ceres::CostFunction {
public:
  // 预积分测量值和时间间隔
  Eigen::Vector3d alpha;
  Eigen::Vector3d beta;
  Eigen::Vector4d q_breve;
  double dt;

  // 预积分线性化点
  Eigen::Vector3d b_w_lin_save;
  Eigen::Vector3d b_a_lin_save;

  // 预积分偏差雅可比矩阵
  Eigen::Matrix3d J_q; // J_q - 姿态相对于偏差 w
  Eigen::Matrix3d J_a; // J_a - 位置相对于偏差 w
  Eigen::Matrix3d J_b; // J_b - 速度相对于偏差 w
  Eigen::Matrix3d H_a; // H_a - 位置相对于偏差 a
  Eigen::Matrix3d H_b; // H_b - 速度相对于偏差 a

  // 预积分信息的平方根
  Eigen::Matrix<double, 15, 15> sqrtI_save;

  // 重力
  Eigen::Vector3d grav_save;

  /**
   * @brief 默认构造函数
   */
  Factor_ImuCPIv1(double deltatime, Eigen::Vector3d &grav, Eigen::Vector3d &alpha, Eigen::Vector3d &beta, Eigen::Vector4d &q_KtoK1,
                  Eigen::Vector3d &ba_lin, Eigen::Vector3d &bg_lin, Eigen::Matrix3d &J_q, Eigen::Matrix3d &J_beta, Eigen::Matrix3d &J_alpha,
                  Eigen::Matrix3d &H_beta, Eigen::Matrix3d &H_alpha, Eigen::Matrix<double, 15, 15> &covariance);

  virtual ~Factor_ImuCPIv1() {}

  /**
   * @brief 误差残差和雅可比矩阵计算
   *
   * 这计算积分预积分测量值与当前状态估计之间的误差。
   * 这也考虑了偏差线性化点的变化。
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_IMUCPIV1_H