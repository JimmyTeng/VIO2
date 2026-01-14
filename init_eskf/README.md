# Init-ESKF 独立模块

## 概述

`init_eskf` 是一个独立的 ROS 包，提供基于误差状态卡尔曼滤波器（ESKF）的 VINS 初始化系统。该模块与 `vins_estimator` 平行，可以独立编译和运行。

## 目录结构

```
init_eskf/
├── CMakeLists.txt          # 编译配置
├── package.xml            # ROS 包配置
├── README.md              # 本文档
├── include/
│   └── init_eskf/
│       ├── init_eskf.h           # ESKF 核心类
│       ├── gt_reader.h            # 真值数据读取器
│       ├── init_eskf_node.h      # ROS 节点类
│       └── utility.h              # 工具函数
├── src/
│   ├── init_eskf.cpp             # ESKF 实现
│   ├── gt_reader.cpp              # 真值读取器实现
│   ├── init_eskf_node.cpp        # ROS 节点实现
│   └── utility/
│       └── utility.cpp           # 工具函数实现
└── launch/
    └── init_eskf.launch          # Launch 文件
```

## 编译

```bash
cd /home/jimmy/project/vio_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 基本使用

```bash
# EuRoC 数据集
roslaunch init_eskf init_eskf.launch \
  gt_type:=euroc \
  gt_path:=/home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
  imu_topic:=/imu0

# KITTI 数据集
roslaunch init_eskf init_eskf.launch \
  gt_type:=kitti \
  gt_path:=/path/to/kitti/2011_10_03/2011_10_03_drive_0027_sync \
  imu_topic:=/imu0
```

### 在代码中使用

```cpp
#include <init_eskf/init_eskf.h>
#include <init_eskf/gt_reader.h>

// 创建 ESKF 实例
init_eskf::InitESKF eskf;
eskf.initialize(9.81, 0.01, 0.001, 1e-5, 1e-6);

// IMU 传播
eskf.propagate(timestamp, acc, gyr);

// TOF 更新
double height;
if (gt_reader.getHeightAtTime(timestamp, height, 0.1)) {
    eskf.updateTOF(timestamp, height, 0.05);
}

// 光流更新
Eigen::Vector2d flow_vel(0.5, 0.3);
eskf.updateFlow(timestamp, flow_vel, 0.1);

// 检查收敛
if (eskf.checkConvergence()) {
    init_eskf::InitESKF::NominalState state = eskf.getState();
    // 使用初始化后的状态
}
```

## 功能特性

- ✅ 独立的 ROS 包，不依赖 vins_estimator
- ✅ 完整的 ESKF 实现
- ✅ 支持 KITTI/EuRoC/通用格式真值数据
- ✅ TOF 高度更新（1D量测）
- ✅ 光流速度更新（2D量测，已修正坐标系转换）
- ✅ 状态收敛检查
- ✅ ROS 节点和库两种使用方式

## 发布的话题

- `/init_eskf/odometry`: 里程计信息
- `/init_eskf/path`: 路径信息
- `/init_eskf/converged`: 收敛状态
- `/init_eskf/uncertainty`: 位置不确定性

## 与 vins_estimator 的关系

- **独立模块**: `init_eskf` 是完全独立的包，不依赖 `vins_estimator`
- **可复用**: 其他项目也可以使用这个初始化模块
- **接口清晰**: 通过 ROS 话题或 C++ API 提供接口

## 更多信息

详细文档请参考：
- API 文档: 查看头文件注释
- 使用示例: `launch/init_eskf.launch`
