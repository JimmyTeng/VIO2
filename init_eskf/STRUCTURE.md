# Init-ESKF 独立模块结构说明

## 模块位置

```
VINS-Fusion/
├── vins_estimator/          # 原有的 VINS 估计器模块
└── init_eskf/               # 新的独立初始化模块（平行结构）
```

## 目录结构

```
init_eskf/
├── CMakeLists.txt           # 编译配置
├── package.xml              # ROS 包配置
├── README.md                # 使用说明
├── STRUCTURE.md             # 本文档
│
├── include/                 # 头文件目录
│   └── init_eskf/
│       ├── init_eskf.h           # ESKF 核心类
│       ├── gt_reader.h           # 真值数据读取器
│       ├── init_eskf_node.h      # ROS 节点类
│       └── utility.h             # 工具函数
│
├── src/                     # 源文件目录
│   ├── init_eskf.cpp            # ESKF 实现
│   ├── gt_reader.cpp             # 真值读取器实现
│   ├── init_eskf_node.cpp        # ROS 节点实现
│   └── utility/
│       └── utility.cpp           # 工具函数实现
│
└── launch/                  # Launch 文件目录
    └── init_eskf.launch          # 主 Launch 文件
```

## 命名空间

所有类都在 `init_eskf` 命名空间中：

```cpp
namespace init_eskf {
    class InitESKF { ... };
    class GroundTruthReader { ... };
    class InitESKFNode { ... };
    class Utility { ... };
}
```

## 使用方式

### 1. 作为 ROS 节点运行

```bash
roslaunch init_eskf init_eskf.launch \
  gt_type:=euroc \
  gt_path:=/path/to/groundtruth.csv \
  imu_topic:=/imu0
```

### 2. 作为库使用

```cpp
#include <init_eskf/init_eskf.h>
#include <init_eskf/gt_reader.h>

init_eskf::InitESKF eskf;
init_eskf::GroundTruthReader gt_reader;
```

## 与 vins_estimator 的关系

- **完全独立**: 不依赖 `vins_estimator` 包
- **可复用**: 其他项目也可以使用
- **接口清晰**: 通过 ROS 话题或 C++ API 提供接口

## 编译

```bash
cd /home/jimmy/project/vio_ws
catkin_make
source devel/setup.bash
```

## 依赖

- ROS (roscpp, std_msgs, geometry_msgs, nav_msgs, sensor_msgs, tf)
- Eigen3
- C++11

## 主要功能

1. **Init-ESKF 核心算法**
   - IMU 积分/预测
   - TOF 高度更新（1D）
   - 光流速度更新（2D，已修正坐标系转换）
   - 状态收敛检查

2. **真值数据读取**
   - KITTI 格式
   - EuRoC 格式
   - 通用格式

3. **ROS 节点**
   - 订阅 IMU 数据
   - 发布初始化结果
   - TF 变换
