# EuRoC 数据集测试说明

## 测试脚本

已创建测试脚本用于测试 Init-ESKF 在 EuRoC V1_01_easy 数据集上的表现。

## 使用方法

### 方法 1: 使用 Shell 脚本（推荐）

```bash
cd /home/jimmy/project/vio_ws
source devel/setup.bash

# 使用默认路径
./src/VINS-Fusion/init_eskf/scripts/test_euroc.sh

# 或指定数据集路径
./src/VINS-Fusion/init_eskf/scripts/test_euroc.sh /path/to/V1_01_easy
```

### 方法 2: 直接运行可执行文件

```bash
cd /home/jimmy/project/vio_ws
source devel/setup.bash

# 使用默认路径
rosrun init_eskf euroc_test

# 或指定数据集路径
rosrun init_eskf euroc_test /home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy
```

## 测试内容

测试脚本会执行以下步骤：

1. **加载数据集**
   - 解析真值数据（ground truth）
   - 解析 IMU 数据
   - 初始点归一化（时间戳和位置都从 0 开始）

2. **初始化 ESKF**
   - 设置噪声参数
   - 初始化协方差矩阵

3. **重力对齐**
   - 使用前 2 秒的 IMU 数据
   - 计算初始姿态
   - 初始化 IMU 偏置

4. **处理数据**
   - IMU 传播（200Hz）
   - TOF 高度更新（50Hz，使用真值高度）
   - 每 1 秒输出一次状态

5. **检查收敛**
   - 检查位置、速度、姿态不确定性
   - 与真值数据比较
   - 输出最终状态

## 输出示例

```
========================================
EuRoC Dataset Test: V1_01_easy
========================================
Dataset path: /home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy

[1/5] Loading dataset...
  ✓ Ground truth data: 28713 points
  ✓ IMU data: 29112 points
  ✓ Initial timestamp: 1403715273.302 s
  ✓ Initial position: [0.879, 2.142, 0.947] m

[2/5] Initializing ESKF...
  ✓ ESKF initialized

[3/5] Performing gravity alignment...
  ✓ Gravity alignment successful
    Initial orientation: [0.0605, -0.8285, -0.0590, -0.5536]

[4/5] Processing IMU and ground truth data...
  Processing from 0.00 s to 10.00 s
  t=1.00s: pos=[0.001, 0.002, 0.000] m, vel=[0.010, -0.014, -0.002] m/s, uncertainty: pos=0.1234m, vel=0.0567m/s, ori=0.0123rad
  ...

[5/5] Checking convergence and final state...
Final State:
  Position: [0.123, 0.456, 0.789] m
  Velocity: [0.012, -0.034, 0.005] m/s
  ...
Convergence: ✓ YES
```

## 参数说明

测试脚本使用的参数：

- **重力对齐持续时间**: 2.0 秒
- **最大加速度方差**: 0.1（用于检测静止状态）
- **TOF 更新频率**: 50Hz（每 0.02 秒）
- **TOF 噪声**: 0.05 m
- **收敛阈值**: 
  - 位置: 0.1 m
  - 速度: 0.1 m/s
  - 姿态: 0.05 rad

## 注意事项

1. **初始点归一化**: 所有时间戳和位置都相对于第一个数据点
2. **数据同步**: 真值数据和 IMU 数据的时间戳已同步归一化
3. **处理时间**: 默认处理前 10 秒的数据，可以修改代码中的 `end_time` 来调整

## 故障排除

如果遇到问题：

1. **数据集路径错误**: 检查路径是否正确
2. **文件不存在**: 确保 `data.csv` 文件存在
3. **编译错误**: 运行 `catkin_make` 重新编译
