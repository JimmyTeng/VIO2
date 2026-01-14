/*******************************************************
 * EuRoC 数据集解析示例
 * 
 * 演示如何使用 EuroCParser 的简化接口
 * 使用 setDatasetPath 和 popNext 接口按时间顺序读取数据
 * 
 * 用法: ./euroc_example [数据集路径] [显示数量]
 * 示例: ./euroc_example /path/to/MH_01_easy 50
 *******************************************************/

#include "init_eskf/euroc_parser.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <utility>
#include <cstdint>

void printUsage(const char* program_name) {
    std::cout << "用法: " << program_name << " [数据集路径] [显示数量]" << std::endl;
    std::cout << "示例: " << program_name << " /path/to/MH_01_easy 50" << std::endl;
    std::cout << std::endl;
    std::cout << "参数说明:" << std::endl;
    std::cout << "  数据集路径: EuRoC 数据集根目录（默认: /home/jimmy/project/vio_ws/src/data/EuRoC/MH_01_easy）" << std::endl;
    std::cout << "  显示数量:   要显示的数据条数（默认: 20）" << std::endl;
}

std::string formatTimestamp(int64_t ts_ns) {
    // 将纳秒转换为秒显示
    double ts_s = ts_ns / 1e9;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << ts_s << "s (" << ts_ns << "ns)";
    return oss.str();
}

int main(int argc, char** argv) {
    // 默认参数
    std::string dataset_path = "/home/jimmy/project/vio_ws/src/data/EuRoC/MH_01_easy";
    size_t display_count = 20;
    
    // 解析命令行参数
    if (argc > 1) {
        if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        dataset_path = argv[1];
    }
    
    if (argc > 2) {
        display_count = std::stoul(argv[2]);
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "EuRoC 数据集解析示例" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "数据集路径: " << dataset_path << std::endl;
    std::cout << "显示数量: " << display_count << std::endl;
    std::cout << std::endl;
    
    // 创建解析器
    init_eskf::EuroCParser parser;
    
    // 设置数据集路径
    std::cout << "正在加载数据集..." << std::endl;
    if (!parser.setDatasetPath(dataset_path)) {
        std::cerr << "错误: 无法设置数据集路径或加载数据失败" << std::endl;
        std::cerr << "请检查数据集路径是否正确" << std::endl;
        return -1;
    }
    std::cout << "数据集加载成功！" << std::endl;
    std::cout << std::endl;
    
    // 统计信息
    size_t image_count = 0;
    size_t imu_count = 0;
    size_t gt_count = 0;
    size_t stereo_count = 0;
    size_t mono_count = 0;
    
    int64_t first_ts = 0;
    int64_t last_ts = 0;
    bool has_first_ts = false;
    
    // 时间戳单调性检查（允许相等，不允许倒退）
    int64_t prev_ts = 0;
    bool has_prev_ts = false;
    size_t non_monotonic_count = 0;
    std::vector<std::pair<size_t, int64_t>> non_monotonic_entries;
    
    // 按时间顺序弹出数据
    size_t total_count = 0;
    size_t displayed = 0;
    
    std::cout << "开始按时间顺序读取数据..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    while (auto result = parser.popNext()) {
        int64_t ts_ns = result->getTimestamp();
        total_count++;
        
        // 检查时间戳是否单调递增（允许相等，不允许倒退）
        if (has_prev_ts) {
            if (ts_ns < prev_ts) {
                // 时间戳倒退
                non_monotonic_count++;
                if (non_monotonic_entries.size() < 10) {
                    non_monotonic_entries.push_back({total_count, ts_ns});
                }
            }
        }
        prev_ts = ts_ns;
        has_prev_ts = true;
        
        switch (result->type) {
            case init_eskf::DataType::IMAGE: {
                const auto& image_data = result->getImage();
                image_count++;
                if (!image_data.filename0.empty() && !image_data.filename1.empty()) {
                    stereo_count++;
                } else {
                    mono_count++;
                }
                
                if (displayed < display_count) {
                    if (!image_data.filename0.empty() && !image_data.filename1.empty()) {
                        std::cout << "[" << std::setw(6) << total_count << "] "
                                  << "双目图像 "
                                  << "t=" << formatTimestamp(ts_ns) << ", "
                                  << "cam0=" << image_data.filename0 
                                  << ", cam1=" << image_data.filename1 << std::endl;
                    } else {
                        std::string cam_info;
                        if (!image_data.filename0.empty()) {
                            cam_info = "cam0=" + image_data.filename0;
                        } else if (!image_data.filename1.empty()) {
                            cam_info = "cam1=" + image_data.filename1;
                        }
                        std::cout << "[" << std::setw(6) << total_count << "] "
                                  << "单目图像 "
                                  << "t=" << formatTimestamp(ts_ns) << ", "
                                  << cam_info << std::endl;
                    }
                    displayed++;
                }
                break;
            }
                
            case init_eskf::DataType::IMU: {
                const auto& imu_data = result->getIMU();
                imu_count++;
                
                if (displayed < display_count) {
                    std::cout << "[" << std::setw(6) << total_count << "] "
                              << "IMU    "
                              << "t=" << formatTimestamp(ts_ns) << ", "
                              << "acc=[" << std::fixed << std::setprecision(3)
                              << imu_data.acc.x() << ", " << imu_data.acc.y() << ", " << imu_data.acc.z() << "], "
                              << "gyr=[" << imu_data.gyr.x() << ", " << imu_data.gyr.y() << ", " << imu_data.gyr.z() << "]"
                              << std::endl;
                    displayed++;
                }
                break;
            }
                
            case init_eskf::DataType::GROUND_TRUTH: {
                const auto& gt_data = result->getGroundTruth();
                gt_count++;
                
                if (displayed < display_count) {
                    std::cout << "[" << std::setw(6) << total_count << "] "
                              << "真值   "
                              << "t=" << formatTimestamp(ts_ns) << ", "
                              << "pos=[" << std::fixed << std::setprecision(3)
                              << gt_data.x << ", " << gt_data.y << ", " << gt_data.z << "], "
                              << "vel=[" << gt_data.vx << ", " << gt_data.vy << ", " << gt_data.vz << "]"
                              << std::endl;
                    displayed++;
                }
                break;
            }
        }
        
        if (!has_first_ts) {
            first_ts = ts_ns;
            has_first_ts = true;
        }
        last_ts = ts_ns;
    }
    
    if (total_count > display_count) {
        std::cout << "..." << std::endl;
        std::cout << "(仅显示前 " << display_count << " 条数据，共 " << total_count << " 条)" << std::endl;
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << std::endl;
    
    // 显示统计结果
    std::cout << "数据统计:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  图像数据:" << std::endl;
    std::cout << "    - 双目: " << stereo_count << " 条" << std::endl;
    std::cout << "    - 单目: " << mono_count << " 条" << std::endl;
    std::cout << "    - 总计: " << image_count << " 条" << std::endl;
    std::cout << "  IMU 数据: " << imu_count << " 条" << std::endl;
    std::cout << "  真值数据: " << gt_count << " 条" << std::endl;
    std::cout << "  总数据点: " << total_count << " 条" << std::endl;
    std::cout << std::endl;
    
    // 显示时间戳单调性检查结果
    std::cout << "时间戳单调性检查:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    if (non_monotonic_count == 0) {
        std::cout << "  ✓ 时间戳单调递增（允许相等）: 通过" << std::endl;
    } else {
        std::cout << "  ✗ 时间戳单调递增（允许相等）: 失败" << std::endl;
        std::cout << "    时间戳倒退次数: " << non_monotonic_count << " 次" << std::endl;
        if (!non_monotonic_entries.empty()) {
            std::cout << "    前 " << non_monotonic_entries.size() << " 个倒退位置:" << std::endl;
            for (const auto& entry : non_monotonic_entries) {
                std::cout << "      位置 " << entry.first << ": " << formatTimestamp(entry.second) << std::endl;
            }
        }
    }
    std::cout << std::endl;
    
    if (has_first_ts) {
        int64_t duration_ns = last_ts - first_ts;
        double duration_s = duration_ns / 1e9;
        std::cout << "时间信息:" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "  起始时间: " << formatTimestamp(first_ts) << std::endl;
        std::cout << "  结束时间: " << formatTimestamp(last_ts) << std::endl;
        std::cout << "  持续时间: " << std::fixed << std::setprecision(6) << duration_s << " s (" 
                  << std::fixed << std::setprecision(2) << (duration_s / 60.0) << " 分钟)" << std::endl;
        std::cout << std::endl;
        
        // 计算频率
        if (image_count > 0 && duration_s > 0) {
            double image_freq = image_count / duration_s;
            std::cout << "数据频率:" << std::endl;
            std::cout << "----------------------------------------" << std::endl;
            std::cout << "  图像频率: " << std::fixed << std::setprecision(2) << image_freq << " Hz" << std::endl;
        }
        if (imu_count > 0 && duration_s > 0) {
            double imu_freq = imu_count / duration_s;
            std::cout << "  IMU频率: " << std::fixed << std::setprecision(2) << imu_freq << " Hz" << std::endl;
        }
        if (gt_count > 0 && duration_s > 0) {
            double gt_freq = gt_count / duration_s;
            std::cout << "  真值频率: " << std::fixed << std::setprecision(2) << gt_freq << " Hz" << std::endl;
        }
    }
    
    std::cout << "========================================" << std::endl;
    
    return 0;
}
