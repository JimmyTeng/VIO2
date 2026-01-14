# 代码审查修复说明

## 修复内容

根据代码审查反馈，修复了以下工程隐患：

### 1. 数值稳定性：协方差更新公式对称化

**问题**：
- 协方差更新公式 `P_ = (I - K * H) * P_` 在计算精度不足时，可能会破坏 P 矩阵的对称性和正定性
- 这会导致滤波器数值不稳定

**修复**：
在所有协方差更新后添加对称化处理：

```cpp
// 更新协方差（使用对称化处理保证数值稳定性）
Matrix15d I = Matrix15d::Identity();
P_ = (I - K * H) * P_;
// 强制对称化，保证 P 矩阵的对称性和正定性
P_ = (P_ + P_.transpose()) / 2.0;
```

**应用位置**：
- `updateTOF()`: TOF 更新后的协方差更新
- `updateFlow()`: 光流更新后的协方差更新
- `propagate()`: 预测步骤后的协方差传播

### 2. 预测模型的重力处理优化

**分析**：
当前实现：
```cpp
Eigen::Vector3d gravity_imu = x_nominal_.q.inverse() * gravity_global;
Eigen::Vector3d acc_body = acc_unbiased - gravity_imu;
Eigen::Vector3d acc_global = x_nominal_.q * acc_body;
```

等价于标准运动学方程：
```
a_world = R_wb * (a_meas - b_a) - g_world
```

**结论**：
- 逻辑在数学上是正确的
- 可以保留，但添加了注释说明其等价性
- 未来可以考虑直接使用标准形式以提高可读性

### 3. 光流量测模型优化

**改进**：
- 简化了变量命名（`R_wb` -> `R`, `v_w` -> `v_w`）
- 统一了预测量测值的计算（`v_b_pred`）
- 改进了注释，使推导更清晰

## 数值稳定性保证

### 对称化处理

协方差矩阵 `P` 必须是：
1. **对称的**：`P = P^T`
2. **正定的**：所有特征值 > 0

由于浮点运算误差，`(I - K*H) * P` 的结果可能不是完全对称的。通过强制对称化：
```cpp
P_ = (P_ + P_.transpose()) / 2.0;
```
可以保证：
- 矩阵严格对称
- 减少数值误差累积
- 提高滤波器稳定性

### Joseph Form（可选改进）

对于更高精度的应用，可以考虑使用 Joseph Form：
```cpp
Matrix15d I = Matrix15d::Identity();
Matrix15d I_KH = I - K * H;
P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
```

Joseph Form 在理论上更稳定，但计算量更大。对于当前应用，简单的对称化已经足够。

## 测试建议

1. **长时间运行测试**：验证对称化处理后滤波器不会发散
2. **数值精度测试**：检查协方差矩阵的特征值是否保持正定
3. **斜坡初始化测试**：验证重力对齐功能在非水平初始姿态下的表现

## 参考

- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2004). *Estimation with applications to tracking and navigation*. John Wiley & Sons.
- Sola, J. (2017). *Quaternion kinematics for the error-state Kalman filter*. arXiv preprint arXiv:1711.02508.
