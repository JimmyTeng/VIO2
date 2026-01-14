# EuRoC 数据集解析器使用说明

## 概述

`EuroCParser` 是专门为 EuRoC 数据集设计的解析器，支持：
- 自动解析真值数据（ground truth）
- 自动解析 IMU 数据
- **初始点归一化**：将第一个时间戳设为 0，第一个位置设为 (0,0,0)

## 数据集结构

EuRoC 数据集结构：
```
V1_01_easy/
├── mav0/
│   ├── state_groundtruth_estimate0/
│   │   └── data.csv          # 真值数据
│   └── imu0/
│       └── data.csv          # IMU 数据
```

## 使用方法

### 1. 基本使用（初始点归一化为 0）

```cpp
#include "init_eskf/euroc_parser.h"

init_eskf::EuroCParser parser;

// 加载数据集，初始点归一化为 0
std::string dataset_path = "/home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy";
if (parser.loadDataset(dataset_path, true, true)) {
    // 成功加载
    // 第一个时间戳 = 0.0
    // 第一个位置 = (0, 0, 0)
}
```

### 2. 获取归一化后的数据

```cpp
// 获取 t=0 时刻的真值数据（归一化后）
init_eskf::GroundTruthData gt;
if (parser.getGroundTruthAtTime(0.0, gt, 0.01)) {
    // gt.timestamp ≈ 0.0
    // gt.x ≈ 0.0, gt.y ≈ 0.0, gt.z ≈ 0.0
}

// 获取 t=1.0s 时刻的 IMU 数据
init_eskf::IMUData imu;
if (parser.getIMUAtTime(1.0, imu, 0.005)) {
    // imu.timestamp ≈ 1.0
    // 使用 imu.acc 和 imu.gyr
}
```

### 3. 获取原始初始值

```cpp
// 获取归一化前的初始时间戳
double original_timestamp = parser.getInitialTimestamp();

// 获取归一化前的初始位置
Eigen::Vector3d original_position = parser.getInitialPosition();
```

### 4. 批量处理

```cpp
// 获取所有真值数据
const auto& all_gt = parser.getGroundTruthData();
for (const auto& gt : all_gt) {
    // 处理每个真值数据点
    // gt.timestamp 从 0 开始
    // gt.x, gt.y, gt.z 从 (0,0,0) 开始
}

// 获取所有 IMU 数据
const auto& all_imu = parser.getIMUData();
for (const auto& imu : all_imu) {
    // 处理每个 IMU 数据点
}
```

## 运行示例程序

编译后运行示例：

```bash
cd /home/jimmy/project/vio_ws
catkin_make
source devel/setup.bash

# 运行示例
rosrun init_eskf euroc_parser_example /home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy
```

## 归一化说明

### 时间戳归一化

- **归一化前**：时间戳为纳秒级大数（例如：1403715274302142976 ns）
- **归一化后**：第一个时间戳 = 0.0 s，后续时间戳相对于第一个

### 位置归一化

- **归一化前**：位置为绝对坐标（例如：[0.878612, 2.142470, 0.947262] m）
- **归一化后**：第一个位置 = [0, 0, 0] m，后续位置相对于第一个

## 数据格式

### 真值数据格式

EuRoC 真值 CSV 格式：
```
timestamp, px, py, pz, qw, qx, qy, qz, vx, vy, vz, ...
```

归一化后：
- `timestamp` 从 0 开始
- `px, py, pz` 从 (0, 0, 0) 开始

### IMU 数据格式

EuRoC IMU CSV 格式：
```
timestamp, wx, wy, wz, ax, ay, az
```

归一化后：
- `timestamp` 从 0 开始（与真值数据同步）

## 与 Init-ESKF 集成

```cpp
#include "init_eskf/euroc_parser.h"
#include "init_eskf/init_eskf.h"

// 1. 加载数据集
init_eskf::EuroCParser parser;
parser.loadDataset(dataset_path, true, true);  // 初始点归一化为 0

// 2. 初始化 ESKF
init_eskf::InitESKF eskf;
eskf.initialize(9.81, 0.01, 0.001, 1e-5, 1e-6);

// 3. 处理 IMU 数据
const auto& imu_data = parser.getIMUData();
for (const auto& imu : imu_data) {
    eskf.propagate(imu.timestamp, imu.acc, imu.gyr);
    
    // 4. TOF 更新（使用归一化后的高度）
    init_eskf::GroundTruthData gt;
    if (parser.getGroundTruthAtTime(imu.timestamp, gt, 0.01)) {
        eskf.updateTOF(imu.timestamp, gt.z, 0.05);
    }
}
```

## 注意事项

1. **时间同步**：真值数据和 IMU 数据的时间戳都已归一化，可以直接使用
2. **初始位置**：归一化后第一个位置为 (0,0,0)，适合初始化 ESKF
3. **时间戳精度**：EuRoC 使用纳秒级时间戳，解析时转换为秒
4. **数据对齐**：使用 `max_time_diff` 参数来查找最近的数据点
