# OpenVINS Feature Publisher

基于OpenVINS TrackKLT的特征跟踪发布节点，兼容VINS-Mono的消息格式。

## 功能

- 使用OpenVINS的TrackKLT算法进行特征跟踪
- 发布与VINS-Mono相同格式的feature消息 (`sensor_msgs::PointCloud`)
- 支持单目和双目相机
- 支持radtan和equidistant畸变模型
- 频率控制和可视化选项

## 编译

```bash
cd ~/project/vio_ws
catkin_make --only-pkg-with-deps ov_feature_pub
```

## 使用方法

### 1. 配置文件

确保配置文件中包含以下参数：

```yaml
image_topic: "/cam0/image_raw"
camera_calib_file: "/path/to/camera_calib.yaml"
max_cnt: 150
min_dist: 30
freq: 10
show_track: 0
```

### 2. 相机标定文件格式

相机标定YAML文件应包含：

```yaml
resolution: [752, 480]  # [width, height]
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_coefficients:
  rows: 4
  cols: 1
  data: [k1, k2, p1, p2]
distortion_model: "radtan"  # or "equidistant"
```

### 3. 运行节点

```bash
# 方法1: 使用launch文件
roslaunch ov_feature_pub ov_feature_pub.launch config_path:=/path/to/config.yaml

# 方法2: 直接运行
rosrun ov_feature_pub ov_feature_pub _config_file:=/path/to/config.yaml
```

### 4. 参数说明

- `config_file`: 配置文件路径（必须）
- `max_features`: 最大特征数量（默认: 150）
- `fast_threshold`: FAST角点检测阈值（默认: 20）
- `grid_x`: X方向网格数量（默认: 5）
- `grid_y`: Y方向网格数量（默认: 5）
- `min_px_dist`: 特征点最小像素距离（默认: 10）
- `equalize`: 是否使用直方图均衡化（默认: false）
- `distortion_model`: 畸变模型类型，radtan或equidistant（默认: radtan）

## 发布的话题

- `/feature_tracker/feature` (sensor_msgs::PointCloud): 特征点数据
  - points: 归一化坐标 (x, y, z=1)
  - channels[0]: 特征ID
  - channels[1]: 像素坐标u
  - channels[2]: 像素坐标v
  - channels[3]: 速度vx
  - channels[4]: 速度vy

- `/feature_tracker/feature_img` (sensor_msgs::Image): 可视化图像（如果show_track=1）

- `/feature_tracker/restart` (std_msgs::Bool): 重启标志

## 与VINS-Mono的兼容性

该节点发布的消息格式与VINS-Mono的`feature_tracker`节点完全兼容，可以直接替换使用：

```xml
<!-- 替换前 -->
<node name="feature_tracker" pkg="feature_tracker" type="feature_tracker" />

<!-- 替换后 -->
<node name="ov_feature_pub" pkg="ov_feature_pub" type="ov_feature_pub" />
```

## 注意事项

1. 确保`ov_core`包已正确编译
2. 相机标定文件路径必须正确
3. 图像话题名称需要与配置文件中的`image_topic`一致
4. 特征点只有在被跟踪至少2帧后才会发布（与VINS-Mono行为一致）
