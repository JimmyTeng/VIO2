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

#ifndef OV_INIT_CERES_JPLQUATLOCAL_H
#define OV_INIT_CERES_JPLQUATLOCAL_H

#include <ceres/ceres.h>

#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2
#include <ceres/manifold.h>
#endif

namespace ov_init {

/**
 * @brief JPL 四元数 CERES 状态参数化
 */
#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2
class State_JPLQuatLocal : public ceres::Manifold {
#else
class State_JPLQuatLocal : public ceres::LocalParameterization {
#endif
public:
  /**
   * @brief JPL 四元数表示的状态更新函数。
   *
   * 通过将当前四元数与从小轴角扰动构建的四元数左乘来实现更新操作。
   *
   * @f[
   * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
   * @f]
   */
  bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;

#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2

  bool PlusJacobian(const double *x, double *jacobian) const override;

  // 逆更新：delta = Log(q2 ⊗ inv(q1))
  bool Minus(const double* y, const double* x, double* delta) const override;

  // Minus 的雅可比矩阵
  bool MinusJacobian(const double* x, double* jacobian) const override;

  int AmbientSize() const override { return 4; }
  int TangentSize() const override { return 3; }

#else

  /**
   * @brief 计算相对于局部参数化的雅可比矩阵
   *
   * 这本质上"欺骗"了 ceres。
   * 而不是做 ceres 想要的：
   * dr/dlocal= dr/dglobal * dglobal/dlocal
   *
   * 我们直接做：
   * dr/dlocal= [ dr/dlocal, 0] * [I; 0]= dr/dlocal.
   * 因此我们在这里定义 dglobal/dlocal= [I; 0]
   */
  bool ComputeJacobian(const double *x, double *jacobian) const override;

  int GlobalSize() const override { return 4; };

  int LocalSize() const override { return 3; };

#endif

};

} // namespace ov_init

#endif // OV_INIT_CERES_JPLQUATLOCAL_H