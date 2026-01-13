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

/**
 * @namespace ov_init
 * @brief 状态初始化代码
 *
 * 目前包含用于视觉惯性系统的 StaticInitializer 和 DynamicInitializer 初始化代码。
 * 它将等待平台静止，然后在重力坐标系中初始化其姿态。
 *
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 *
 * 如果平台不是静止的，则我们利用动态初始化来尝试恢复初始状态。
 * 这是对论文 [Estimator initialization in vision-aided inertial navigation with unknown camera-IMU
 * calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS 的实现，该论文通过首先创建一个
 * 线性系统来恢复速度、重力和特征位置来解决初始化问题。
 * 在初始恢复之后，执行完整的优化以允许协方差恢复。
 * 有关高级概述，请参阅此 [技术报告](https://pgeneva.com/downloads/reports/tr_init.pdf)。
 */
namespace ov_init {}
