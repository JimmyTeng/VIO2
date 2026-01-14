#!/bin/bash

# EuRoC 解析函数测试脚本
# 仅测试 EuroCParser，对数据进行读取与归一化检查

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 默认数据集路径
DATASET_PATH="/home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy"

# 如果提供了参数，使用参数作为数据集路径
if [ $# -gt 0 ]; then
    DATASET_PATH="$1"
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}EuRoC Parser Test Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo "Dataset path: $DATASET_PATH"
echo ""

# 检查数据集路径是否存在
if [ ! -d "$DATASET_PATH" ]; then
    echo -e "${RED}Error: Dataset path does not exist!${NC}"
    echo "Path: $DATASET_PATH"
    exit 1
fi

# 检查必要文件
GT_FILE="$DATASET_PATH/mav0/state_groundtruth_estimate0/data.csv"
IMU_FILE="$DATASET_PATH/mav0/imu0/data.csv"

if [ ! -f "$GT_FILE" ]; then
    echo -e "${RED}Error: Ground truth file not found!${NC}"
    echo "Expected: $GT_FILE"
    exit 1
fi

if [ ! -f "$IMU_FILE" ]; then
    echo -e "${RED}Error: IMU file not found!${NC}"
    echo "Expected: $IMU_FILE"
    exit 1
fi

echo -e "${GREEN}✓ Dataset files found${NC}"
echo ""

# 设置 ROS 环境
echo "[test_euroc_parser.sh] 设置 ROS 环境..."
source /opt/ros/melodic/setup.bash 2>/dev/null || source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/project/vio_ws/devel/setup.bash
echo "[test_euroc_parser.sh] ROS 环境设置完成"

echo -e "${YELLOW}Running EuroCParser example...${NC}"
echo ""

# 尝试直接运行可执行文件（如果 rosrun 有问题）
EXECUTABLE_PATH="$HOME/project/vio_ws/devel/lib/init_eskf/euroc_parser_example"
if [ -f "$EXECUTABLE_PATH" ]; then
    echo "[test_euroc_parser.sh] 找到可执行文件: $EXECUTABLE_PATH"
    echo "[test_euroc_parser.sh] 直接运行可执行文件..."
    "$EXECUTABLE_PATH" "$DATASET_PATH"
    EXIT_CODE=$?
else
    echo "[test_euroc_parser.sh] 可执行文件不存在，使用 rosrun..."
    echo "[test_euroc_parser.sh] 可执行文件路径: $EXECUTABLE_PATH"
    rosrun init_eskf euroc_parser_example "$DATASET_PATH"
    EXIT_CODE=$?
fi

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Parser test completed successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Parser test failed with exit code: $EXIT_CODE${NC}"
    echo -e "${RED}========================================${NC}"
fi

exit $EXIT_CODE

