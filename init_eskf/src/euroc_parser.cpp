/*******************************************************
 * EuRoC 数据集解析器
 * 
 * 专门用于解析 EuRoC 数据集，支持初始点归一化
 * 针对 V1_01_easy 数据集
 *******************************************************/

#include "init_eskf/euroc_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace init_eskf {

EuroCParser::EuroCParser() : 
    initial_timestamp_(0.0),
    initial_position_(Eigen::Vector3d::Zero()),
    time_normalized_(false),
    position_normalized_(false) {
}

EuroCParser::~EuroCParser() {
}

bool EuroCParser::loadDataset(const std::string &dataset_path,
                              bool normalize_time,
                              bool normalize_position) {
    dataset_path_ = dataset_path;
    time_normalized_ = normalize_time;
    position_normalized_ = normalize_position;
    
    // 构建文件路径
    std::string gt_file = dataset_path + "/mav0/state_groundtruth_estimate0/data.csv";
    std::string imu_file = dataset_path + "/mav0/imu0/data.csv";
    
    // 先读取 IMU 数据的第一个时间戳（用于统一归一化基准）
    double imu_first_timestamp = 0.0;
    if (normalize_time) {
        std::ifstream imu_preview(imu_file);
        if (imu_preview.is_open()) {
            std::string line;
            std::getline(imu_preview, line); // 跳过标题行
            if (std::getline(imu_preview, line) && !line.empty()) {
                std::istringstream iss(line);
                std::string field;
                std::vector<std::string> fields;
                while (std::getline(iss, field, ',')) {
                    fields.push_back(field);
                }
                if (fields.size() >= 1) {
                    imu_first_timestamp = std::stod(fields[0]) * 1e-9; // 纳秒转秒
                }
            }
            imu_preview.close();
        }
    }
    
    // 加载真值数据（使用 IMU 的第一个时间戳作为归一化基准，确保所有时间戳都是正数）
    if (!loadGroundTruth(gt_file, normalize_time, normalize_position, imu_first_timestamp)) {
        std::cerr << "Failed to load ground truth data" << std::endl;
        return false;
    }
    
    // 加载 IMU 数据（使用自己的第一个时间戳作为归一化基准）
    if (!loadIMUData(imu_file, normalize_time, imu_first_timestamp)) {
        std::cerr << "Failed to load IMU data" << std::endl;
        return false;
    }
    
    std::cerr << "[loadDataset] 数据集加载完成" << std::endl;
    std::cerr.flush();
    
    std::cout << "==========================================" << std::endl;
    std::cout << "EuRoC Dataset Loaded Successfully" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Ground truth data points: " << gt_data_.size() << std::endl;
    std::cout << "IMU data points: " << imu_data_.size() << std::endl;
    if (normalize_time) {
        std::cout << "Initial timestamp: " << initial_timestamp_ << " s (normalized to 0)" << std::endl;
    }
    if (normalize_position) {
        std::cout << "Initial position: [" 
                  << initial_position_.x() << ", " 
                  << initial_position_.y() << ", " 
                  << initial_position_.z() << "] m (normalized to [0,0,0])" << std::endl;
    }
    std::cout << "==========================================" << std::endl;
    
    return true;
}

bool EuroCParser::loadGroundTruth(const std::string &csv_file,
                                  bool normalize_time,
                                  bool normalize_position,
                                  double normalize_time_base) {
    std::cerr << "[loadGroundTruth] 开始加载: " << csv_file << std::endl;
    std::cerr.flush();
    
    gt_data_.clear();
    
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << csv_file << std::endl;
        return false;
    }
    
    std::cerr << "[loadGroundTruth] 文件打开成功" << std::endl;
    std::cerr.flush();
    
    std::string line;
    // 跳过标题行
    std::getline(file, line);
    std::cerr << "[loadGroundTruth] 跳过标题行: " << line << std::endl;
    std::cerr.flush();
    
    // 先读取所有数据到临时容器
    std::vector<GroundTruthData> temp_data;
    size_t line_count = 0;
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        ++line_count;
        if (line_count % 10000 == 0) {
            std::cerr << "[loadGroundTruth] 已读取 " << line_count << " 行" << std::endl;
            std::cerr.flush();
        }
        
        GroundTruthData gt;
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;
        
        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }
        
        // EuRoC 格式: timestamp, px, py, pz, qw, qx, qy, qz, vx, vy, vz, ...
        if (fields.size() >= 11) {
            gt.timestamp = std::stod(fields[0]) * 1e-9;  // 纳秒转秒
            gt.x = std::stod(fields[1]);
            gt.y = std::stod(fields[2]);
            gt.z = std::stod(fields[3]);  // 高度
            gt.qw = std::stod(fields[4]);
            gt.qx = std::stod(fields[5]);
            gt.qy = std::stod(fields[6]);
            gt.qz = std::stod(fields[7]);
            gt.vx = std::stod(fields[8]);
            gt.vy = std::stod(fields[9]);
            gt.vz = std::stod(fields[10]);
            
            temp_data.push_back(gt);
        }
    }
    
    file.close();
    
    std::cerr << "[loadGroundTruth] 文件读取完成，共 " << line_count << " 行，有效数据 " << temp_data.size() << " 条" << std::endl;
    std::cerr.flush();
    
    if (temp_data.empty()) {
        std::cerr << "未找到有效的 EuRoC 真值数据" << std::endl;
        return false;
    }
    
    // 获取初始值（第一个数据点）
    initial_timestamp_ = temp_data[0].timestamp;
    initial_position_ = Eigen::Vector3d(temp_data[0].x, temp_data[0].y, temp_data[0].z);
    
    std::cerr << "[loadGroundTruth] 开始归一化 " << temp_data.size() << " 条数据..." << std::endl;
    std::cerr.flush();
    
    // 归一化所有数据
    size_t norm_count = 0;
    for (auto &gt : temp_data) {
        ++norm_count;
        if (norm_count % 10000 == 0) {
            std::cerr << "[loadGroundTruth] 已归一化 " << norm_count << " 条数据" << std::endl;
            std::cerr.flush();
        }
        // 时间戳归一化：使用统一的基准时间戳（通常是 IMU 的第一个时间戳）
        if (normalize_time) {
            if (normalize_time_base > 0.0) {
                gt.timestamp = gt.timestamp - normalize_time_base;
            } else {
                gt.timestamp = gt.timestamp - initial_timestamp_;
            }
            
            // 如果归一化后时间戳小于0（即小于IMU的0点时间），直接丢弃
            if (gt.timestamp < 0.0) {
                continue;
            }
        }
        
        // 位置归一化：减去初始位置
        if (normalize_position) {
            gt.x = gt.x - initial_position_.x();
            gt.y = gt.y - initial_position_.y();
            gt.z = gt.z - initial_position_.z();
        }
        
        // 存储到向量中（按时间戳排序）
        gt_data_.push_back(gt);
    }
    
    std::cerr << "[loadGroundTruth] 归一化完成，共存储 " << gt_data_.size() << " 条数据" << std::endl;
    std::cerr.flush();
    
    return true;
}

bool EuroCParser::loadIMUData(const std::string &csv_file,
                             bool normalize_time,
                             double normalize_time_base) {
    std::cerr << "[loadIMUData] 开始加载: " << csv_file << std::endl;
    std::cerr.flush();
    
    imu_data_.clear();
    
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << csv_file << std::endl;
        return false;
    }
    
    std::cerr << "[loadIMUData] 文件打开成功" << std::endl;
    std::cerr.flush();
    
    std::string line;
    // 跳过标题行
    std::getline(file, line);
    std::cerr << "[loadIMUData] 跳过标题行: " << line << std::endl;
    std::cerr.flush();
    
    // 先读取所有数据
    std::vector<IMUData> temp_data;
    size_t line_count = 0;
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        ++line_count;
        if (line_count % 50000 == 0) {
            std::cerr << "[loadIMUData] 已读取 " << line_count << " 行" << std::endl;
            std::cerr.flush();
        }
        
        IMUData imu;
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;
        
        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }
        
        // EuRoC IMU 格式: timestamp, wx, wy, wz, ax, ay, az
        if (fields.size() >= 7) {
            imu.timestamp = std::stod(fields[0]) * 1e-9;  // 纳秒转秒
            imu.gyr.x() = std::stod(fields[1]);
            imu.gyr.y() = std::stod(fields[2]);
            imu.gyr.z() = std::stod(fields[3]);
            imu.acc.x() = std::stod(fields[4]);
            imu.acc.y() = std::stod(fields[5]);
            imu.acc.z() = std::stod(fields[6]);
            
            temp_data.push_back(imu);
        }
    }
    
    file.close();
    
    std::cerr << "[loadIMUData] 文件读取完成，共 " << line_count << " 行，有效数据 " << temp_data.size() << " 条" << std::endl;
    std::cerr.flush();
    
    if (temp_data.empty()) {
        std::cerr << "未找到有效的 EuRoC IMU 数据" << std::endl;
        return false;
    }
    
    // 时间戳归一化基准：优先使用传入的 normalize_time_base（通常是 IMU 的第一个时间戳）
    // 这样确保所有时间戳都从 0 开始，不会有负数
    double imu_initial_timestamp = 0.0;
    if (normalize_time) {
        if (normalize_time_base > 0.0) {
            imu_initial_timestamp = normalize_time_base;
        } else {
            imu_initial_timestamp = temp_data[0].timestamp;
        }
    } else {
        imu_initial_timestamp = temp_data[0].timestamp;
    }
    
    std::cerr << "[loadIMUData] 开始归一化 " << temp_data.size() << " 条数据..." << std::endl;
    std::cerr.flush();
    
    // 归一化所有数据
    size_t norm_count = 0;
    for (auto &imu : temp_data) {
        ++norm_count;
        if (norm_count % 50000 == 0) {
            std::cerr << "[loadIMUData] 已归一化 " << norm_count << " 条数据" << std::endl;
            std::cerr.flush();
        }
        // 时间戳归一化：使用统一的基准时间戳（IMU 的第一个时间戳）
        if (normalize_time) {
            imu.timestamp = imu.timestamp - imu_initial_timestamp;
        }
        
        // 存储到向量中
        imu_data_.push_back(imu);
    }
    
    std::cerr << "[loadIMUData] 归一化完成，共存储 " << imu_data_.size() << " 条数据" << std::endl;
    std::cerr.flush();
    
    return true;
}

bool EuroCParser::scanGroundTruth(
    const std::string &csv_file,
    const std::function<void(const GroundTruthData&)> &handler) const {
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << csv_file << std::endl;
        return false;
    }

    std::string line;
    // 跳过标题行
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        GroundTruthData gt;
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;

        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }

        // EuRoC 格式: timestamp, px, py, pz, qw, qx, qy, qz, vx, vy, vz, ...
        if (fields.size() >= 11) {
            gt.timestamp = std::stod(fields[0]) * 1e-9;  // 纳秒转秒
            gt.x = std::stod(fields[1]);
            gt.y = std::stod(fields[2]);
            gt.z = std::stod(fields[3]);
            gt.qw = std::stod(fields[4]);
            gt.qx = std::stod(fields[5]);
            gt.qy = std::stod(fields[6]);
            gt.qz = std::stod(fields[7]);
            gt.vx = std::stod(fields[8]);
            gt.vy = std::stod(fields[9]);
            gt.vz = std::stod(fields[10]);

            handler(gt);
        }
    }

    file.close();
    return true;
}

bool EuroCParser::scanIMU(
    const std::string &csv_file,
    const std::function<void(const IMUData&)> &handler) const {
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << csv_file << std::endl;
        return false;
    }

    std::string line;
    // 跳过标题行
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        IMUData imu;
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;

        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }

        // EuRoC IMU 格式: timestamp, wx, wy, wz, ax, ay, az
        if (fields.size() >= 7) {
            imu.timestamp = std::stod(fields[0]) * 1e-9;  // 纳秒转秒
            imu.gyr.x() = std::stod(fields[1]);
            imu.gyr.y() = std::stod(fields[2]);
            imu.gyr.z() = std::stod(fields[3]);
            imu.acc.x() = std::stod(fields[4]);
            imu.acc.y() = std::stod(fields[5]);
            imu.acc.z() = std::stod(fields[6]);

            handler(imu);
        }
    }

    file.close();
    return true;
}

bool EuroCParser::getGroundTruthAtTime(double timestamp, GroundTruthData &gt_data,
                                      double max_time_diff) const {
    if (gt_data_.empty()) {
        return false;
    }
    
    // 使用二分查找（数据已按时间戳排序）
    auto it = std::lower_bound(gt_data_.begin(), gt_data_.end(), timestamp,
        [](const GroundTruthData &gt, double ts) {
            return gt.timestamp < ts;
        });
    
    double best_diff = max_time_diff;
    bool found = false;
    
    // 检查当前元素
    if (it != gt_data_.end()) {
        double diff = std::abs(it->timestamp - timestamp);
        if (diff < best_diff) {
            best_diff = diff;
            gt_data = *it;
            found = true;
        }
    }
    
    // 检查前一个元素（可能更接近）
    if (it != gt_data_.begin()) {
        auto prev_it = std::prev(it);
        double diff = std::abs(prev_it->timestamp - timestamp);
        if (diff < best_diff) {
            gt_data = *prev_it;
            found = true;
        }
    }
    
    return found;
}

bool EuroCParser::getIMUAtTime(double timestamp, IMUData &imu_data,
                               double max_time_diff) const {
    if (imu_data_.empty()) {
        return false;
    }
    
    // 查找最近的数据点
    double best_diff = max_time_diff;
    bool found = false;
    
    for (const auto &imu : imu_data_) {
        double diff = std::abs(imu.timestamp - timestamp);
        if (diff < best_diff) {
            best_diff = diff;
            imu_data = imu;
            found = true;
        }
    }
    
    return found;
}

// 注意：getInitialTimestamp, getInitialPosition, getGroundTruthCount, getIMUCount
// 这些函数在头文件中已定义为内联函数，不需要在这里实现

} // namespace init_eskf
