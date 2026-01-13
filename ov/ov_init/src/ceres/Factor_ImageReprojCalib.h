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

#ifndef OV_INIT_CERES_IMAGEREPROJCALIB_H
#define OV_INIT_CERES_IMAGEREPROJCALIB_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <deque>
#include <iostream>
#include <map>

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "utils/quat_ops.h"

namespace ov_init {

/**
 * @brief 带校准的特征方向观测（原始）因子
 */
class Factor_ImageReprojCalib : public ceres::CostFunction {
public:
  // 特征的测量观测（原始像素坐标）
  Eigen::Vector2d uv_meas;

  // 测量噪声
  double pix_sigma = 1.0;
  Eigen::Matrix<double, 2, 2> sqrtQ;

  // 畸变模型是鱼眼还是径向切向
  bool is_fisheye = false;

  // 如果值为 1，则此残差添加到问题中，否则如果为零，则被"门控"
  double gate = 1.0;

  /**
   * @brief 默认构造函数
   * @param uv_meas_ 环境特征的原始像素 uv 测量值
   * @param pix_sigma_ 原始像素测量不确定性（通常为 1）
   * @param is_fisheye_ 此原始像素相机是否使用鱼眼畸变
   */
  Factor_ImageReprojCalib(const Eigen::Vector2d &uv_meas_, double pix_sigma_, bool is_fisheye_);

  virtual ~Factor_ImageReprojCalib() {}

  /**
   * @brief 误差残差和雅可比矩阵计算
   *
   * 这计算特征投影模型的雅可比矩阵和残差。
   * 这是观测姿态、全局特征和校准参数的函数。
   * 找到归一化像素坐标，然后使用相机畸变模型进行畸变。
   * 有关更多详细信息，请参阅 @ref update-feat 页面。
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_IMAGEREPROJCALIB_H