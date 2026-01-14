# 重力对齐（Gravity Alignment）功能说明

## 问题描述

在原始实现中，`initialize` 函数直接设置 `x_nominal_.q = Identity`，这会导致以下问题：

1. **斜坡初始化失败**：如果无人机放在斜坡上（例如倾斜 5 度），初始重力向量在机体系下不是 `[0,0,9.8]`
2. **错误的重力投影**：在 `propagate` 函数中，`acc_global = q * acc_body` 会错误地把重力分量投影到水平加速上
3. **状态发散**：ESKF 会认为无人机在疯狂横移，导致位置和速度估计发散

## 解决方案

实现了**静态重力对齐**功能，利用前几秒的加速度计读数来初始化姿态四元数：

1. **收集静态数据**：收集前 N 秒（默认 2.0 秒）的加速度计和陀螺仪读数
2. **检测静止状态**：通过计算加速度方差来检测系统是否静止
3. **计算重力方向**：计算平均加速度，归一化得到重力方向
4. **对齐姿态**：使用 `Eigen::Quaterniond::FromTwoVectors` 将机体系的重力方向对齐到全局系的 z 轴
5. **初始化偏置**：同时初始化加速度计和陀螺仪偏置

## 使用方法

### 在 ROS 节点中使用（自动）

重力对齐默认启用。节点会自动收集前 2 秒的 IMU 数据并完成对齐：

```bash
roslaunch init_eskf init_eskf.launch \
  use_gravity_alignment:=true \
  gravity_alignment_duration:=2.0 \
  max_acc_variance_for_alignment:=0.1
```

### 在代码中使用

```cpp
#include <init_eskf/init_eskf.h>

init_eskf::InitESKF eskf;

// 收集 IMU 数据
std::vector<std::pair<double, Eigen::Vector3d>> acc_samples;
std::vector<std::pair<double, Eigen::Vector3d>> gyr_samples;

// ... 收集 2 秒的数据 ...

// 执行重力对齐
if (eskf.initializeWithGravityAlignment(
        acc_samples, 
        gyr_samples,
        2.0,  // 静态对齐持续时间
        0.1)) // 最大加速度方差阈值
{
    // 对齐成功，可以开始使用 ESKF
    eskf.propagate(timestamp, acc, gyr);
} else {
    // 对齐失败（可能系统在运动）
    ROS_WARN("Gravity alignment failed");
}
```

## 参数说明

- `use_gravity_alignment`: 是否启用重力对齐（默认 `true`）
- `gravity_alignment_duration`: 静态对齐持续时间（秒，默认 `2.0`）
- `max_acc_variance_for_alignment`: 最大加速度方差阈值，用于检测静止状态（默认 `0.1`）

## 工作原理

1. **重力对齐算法**：
   ```cpp
   // 计算平均加速度（在机体系中的重力方向）
   Eigen::Vector3d gravity_body = acc_mean.normalized();
   
   // 重力在全局坐标系中指向 z 轴
   Eigen::Vector3d gravity_global(0, 0, gravity_mag_);
   
   // 计算从机体系到全局系的旋转
   Eigen::Quaterniond q_body_to_global = 
       Eigen::Quaterniond::FromTwoVectors(gravity_body, gravity_global.normalized());
   
   // 取逆得到从全局系到机体系的旋转（G to I）
   x_nominal_.q = q_body_to_global.inverse();
   ```

2. **偏置初始化**：
   - 陀螺仪偏置 = 静止时的平均角速度
   - 加速度计偏置 = 平均加速度 - 旋转后的重力

## 注意事项

1. **静止要求**：重力对齐需要系统在静止状态下进行，否则会失败
2. **时间窗口**：建议使用 2-3 秒的静态数据窗口
3. **方差阈值**：如果加速度方差超过阈值，说明系统在运动，对齐会失败
4. **禁用选项**：如果不需要重力对齐（例如已有其他方式初始化姿态），可以设置 `use_gravity_alignment:=false`

## 效果

启用重力对齐后：
- ✅ 可以在斜坡上正确初始化
- ✅ 重力向量被正确解释，不会导致虚假的水平加速度
- ✅ 初始姿态更准确，收敛更快
- ✅ 同时初始化了 IMU 偏置
