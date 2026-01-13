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

#ifndef OV_INIT_CERES_GENERICPRIOR_H
#define OV_INIT_CERES_GENERICPRIOR_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief 用于特定类型的通用状态先验因子。
 *
 * 这是一个通用因子，处理具有非零线性误差的状态先验。
 * 通常，单一因子在创建时误差为零，因此可以忽略这个额外项。
 * 但如果执行边缘化，这可能非零。请参阅以下论文第 3.2 节公式 25-35
 * https://journals.sagepub.com/doi/full/10.1177/0278364919835021
 *
 * 我们有以下最小化问题：
 * @f[
 * \textrm{argmin} ||A * (x - x_{lin}) + b||^2
 * @f]
 *
 *
 * 通常，在边缘化之后，我们有以下内容：
 * - @f$(A^T*A) = Inf_{prior} @f$ (先验信息)
 * - @f$A^T*b = grad_{prior} @f$ (先验梯度)
 *
 * 例如，考虑我们有以下系统，我们希望移除 xm 状态。
 * 这是状态边缘化的问题。
 * @f[
 * [ Arr Arm ] [ xr ] = [ - gr ]
 * @f]
 * @f[
 * [ Amr Amm ] [ xm ] = [ - gm ]
 * @f]
 *
 * 我们希望边缘化与其他状态 @f$ xr @f$ 相关的 xm 状态。
 * 雅可比矩阵（以及信息矩阵 A）在当前最佳猜测 @f$ x_{lin} @f$ 处计算。
 * 我们可以定义以下仅涉及 @f$ xr @f$ 状态的最优子成本形式：
 * @f[
 * cost^2 = (xr - xr_{lin})^T*(A^T*A)*(xr - xr_{lin}) + b^T*A*(xr - xr_{lin}) + b^b
 * @f]
 *
 * 其中我们有：
 * @f[
 * A = sqrt(Arr - Arm*Amm^{-1}*Amr)
 * @f]
 * @f[
 * b = A^-1 * (gr - Arm*Amm^{-1}*gm)
 * @f]
 *
 */
class Factor_GenericPrior : public ceres::CostFunction {
public:
  /// 边缘化时用于线性化问题的状态估计
  Eigen::MatrixXd x_lin;

  /// x_lin 中每个变量的状态类型。可以是 [quat, quat_yaw, vec3, vec8]
  std::vector<std::string> x_type;

  /// 信息的平方根，使得 sqrtI^T * sqrtI = 边缘信息
  Eigen::MatrixXd sqrtI;

  /// 成本函数内的常数项，使得 sqrtI^T * b = 边缘梯度（可以为零）
  Eigen::MatrixXd b;

  /**
   * @brief 默认构造函数
   */
  Factor_GenericPrior(const Eigen::MatrixXd &x_lin_, const std::vector<std::string> &x_type_, const Eigen::MatrixXd &prior_Info,
                      const Eigen::MatrixXd &prior_grad);

  virtual ~Factor_GenericPrior() {}

  /**
   * @brief 误差残差和雅可比矩阵计算
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_GENERICPRIOR_H