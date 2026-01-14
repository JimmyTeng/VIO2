/*******************************************************
 * EuRoC 数据集解析示例
 * 
 * 演示如何使用 EuroCParser 解析 V1_01_easy 数据集
 * 初始点设置为 0
 *******************************************************/

#include "init_eskf/euroc_parser.h"
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    // 数据集路径
    std::string dataset_path = "/home/jimmy/project/vio_ws/src/data/EuRoC/V1_01_easy";
    
    if (argc > 1) {
        dataset_path = argv[1];
    }
    
    // 创建解析器
    init_eskf::EuroCParser parser;

    // 构建 CSV 路径
    std::string gt_file  = dataset_path + "/mav0/state_groundtruth_estimate0/data.csv";
    std::string imu_file = dataset_path + "/mav0/imu0/data.csv";

    std::cout << "Streaming EuRoC dataset from: " << dataset_path << std::endl;
    std::cout << "GT file : " << gt_file << std::endl;
    std::cout << "IMU file: " << imu_file << std::endl;

    // 流式扫描 GroundTruth
    size_t gt_count = 0;
    double gt_first_ts = 0.0;
    double gt_last_ts  = 0.0;
    bool   gt_has_ts   = false;

    std::cout << "Scanning GroundTruth (streaming) ..." << std::endl;
    bool gt_ok = parser.scanGroundTruth(
        gt_file,
        [&](const GroundTruthData &gt) {
            ++gt_count;
            if (!gt_has_ts) {
                gt_first_ts = gt.timestamp;
                gt_has_ts   = true;
            }
            gt_last_ts = gt.timestamp;
        });

    if (!gt_ok) {
        std::cerr << "Failed to stream ground truth from: " << gt_file << std::endl;
        return -1;
    }

    // 流式扫描 IMU
    size_t imu_count = 0;
    double imu_first_ts = 0.0;
    double imu_last_ts  = 0.0;
    bool   imu_has_ts   = false;

    std::cout << "Scanning IMU (streaming) ..." << std::endl;
    bool imu_ok = parser.scanIMU(
        imu_file,
        [&](const init_eskf::IMUData &imu) {
            ++imu_count;
            if (!imu_has_ts) {
                imu_first_ts = imu.timestamp;
                imu_has_ts   = true;
            }
            imu_last_ts = imu.timestamp;
        });

    if (!imu_ok) {
        std::cerr << "Failed to stream IMU from: " << imu_file << std::endl;
        return -1;
    }

    // 显示统计结果
    std::cout << "\n流式数据统计:" << std::endl;
    std::cout << "  真值数据点(流式): " << gt_count << std::endl;
    if (gt_has_ts) {
        std::cout << "  真值时间范围: " << gt_first_ts << " ~ " << gt_last_ts
                  << " s, 持续时间: " << (gt_last_ts - gt_first_ts) << " s" << std::endl;
    }

    std::cout << "  IMU 数据点(流式): " << imu_count << std::endl;
    if (imu_has_ts) {
        std::cout << "  IMU 时间范围: " << imu_first_ts << " ~ " << imu_last_ts
                  << " s, 持续时间: " << (imu_last_ts - imu_first_ts) << " s" << std::endl;
    }

    return 0;
}
