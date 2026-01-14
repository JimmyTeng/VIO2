/*******************************************************
 * Ground Truth Reader: 从数据包读取真值高度
 *******************************************************/

#include "init_eskf/gt_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <iomanip>

GroundTruthReader::GroundTruthReader() {}

GroundTruthReader::~GroundTruthReader() {}

bool GroundTruthReader::loadKITTIGroundTruth(const std::string &data_folder,
                                             const std::string &timestamps_file) {
    clear();
    
    std::string timestamps_path = timestamps_file;
    if (timestamps_path.empty()) {
        timestamps_path = data_folder + "/oxts/timestamps.txt";
    }
    
    // 读取时间戳文件
    std::ifstream ts_file(timestamps_path);
    if (!ts_file.is_open()) {
        std::cerr << "无法打开时间戳文件: " << timestamps_path << std::endl;
        return false;
    }
    
    std::vector<double> timestamps;
    std::string line;
    while (std::getline(ts_file, line)) {
        if (line.empty()) continue;
        
        // KITTI 时间戳格式: YYYY-MM-DD HH:MM:SS.ffffff
        int year, month, day, hour, minute;
        double second;
        if (sscanf(line.c_str(), "%d-%d-%d %d:%d:%lf", 
                   &year, &month, &day, &hour, &minute, &second) == 6) {
            // 转换为秒（从当天开始）
            double timestamp = hour * 3600.0 + minute * 60.0 + second;
            timestamps.push_back(timestamp);
        }
    }
    ts_file.close();
    
    // 读取每个时间戳对应的 GPS 数据
    std::string data_path = data_folder;
    if (data_path.back() != '/') {
        data_path += "/";
    }
    data_path += "oxts/data/";
    
    for (size_t i = 0; i < timestamps.size(); i++) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(10) << i;
        std::string data_file = data_path + ss.str() + ".txt";
        
        std::ifstream gps_file(data_file);
        if (!gps_file.is_open()) {
            continue;
        }
        
        GroundTruthData gt;
        gt.timestamp = timestamps[i];
        
        // KITTI GPS 数据格式: lat lon alt roll pitch yaw ...
        double lat, lon, alt;
        double roll, pitch, yaw;
        double vn, ve, vf, vl, vu;
        
        if (gps_file >> lat >> lon >> alt >> roll >> pitch >> yaw) {
            gt.x = lon;  // 注意：KITTI 使用 lon, lat, alt
            gt.y = lat;
            gt.z = alt;  // 高度
            
            // 读取速度（如果存在）
            if (gps_file >> vn >> ve >> vf >> vl >> vu) {
                gt.vx = ve;
                gt.vy = vn;
                gt.vz = vu;
            }
            
            // 从欧拉角计算四元数（如果需要）
            // 这里简化处理，实际应用中可能需要更精确的转换
            
            gt_data_map_[gt.timestamp] = gt;
        }
        
        gps_file.close();
    }
    
    std::cout << "成功加载 " << gt_data_map_.size() << " 条 KITTI 真值数据" << std::endl;
    return !gt_data_map_.empty();
}

bool GroundTruthReader::loadEuRoCGroundTruth(const std::string &csv_file) {
    clear();
    
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
            gt.z = std::stod(fields[3]);  // 高度
            gt.qw = std::stod(fields[4]);
            gt.qx = std::stod(fields[5]);
            gt.qy = std::stod(fields[6]);
            gt.qz = std::stod(fields[7]);
            gt.vx = std::stod(fields[8]);
            gt.vy = std::stod(fields[9]);
            gt.vz = std::stod(fields[10]);
            
            gt_data_map_[gt.timestamp] = gt;
        }
    }
    
    file.close();
    
    std::cout << "成功加载 " << gt_data_map_.size() << " 条 EuRoC 真值数据" << std::endl;
    return !gt_data_map_.empty();
}

bool GroundTruthReader::loadEuRoCGroundTruthWithOrigin(const std::string &csv_file,
                                                       bool normalize_time,
                                                       bool normalize_position) {
    clear();
    
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << csv_file << std::endl;
        return false;
    }
    
    std::string line;
    // 跳过标题行
    std::getline(file, line);
    
    // 先读取所有数据到临时容器
    std::vector<GroundTruthData> temp_data;
    
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
    
    if (temp_data.empty()) {
        std::cerr << "未找到有效的 EuRoC 数据" << std::endl;
        return false;
    }
    
    // 获取初始值（第一个数据点）
    double initial_timestamp = temp_data[0].timestamp;
    Eigen::Vector3d initial_position(temp_data[0].x, temp_data[0].y, temp_data[0].z);
    
    // 归一化所有数据
    for (auto &gt : temp_data) {
        // 时间戳归一化：减去初始时间戳
        if (normalize_time) {
            gt.timestamp = gt.timestamp - initial_timestamp;
        }
        
        // 位置归一化：减去初始位置
        if (normalize_position) {
            gt.x = gt.x - initial_position.x();
            gt.y = gt.y - initial_position.y();
            gt.z = gt.z - initial_position.z();
        }
        
        // 存储到映射中
        gt_data_map_[gt.timestamp] = gt;
    }
    
    std::cout << "成功加载 " << gt_data_map_.size() << " 条 EuRoC 真值数据" << std::endl;
    if (normalize_time) {
        std::cout << "  时间戳已归一化，初始时间戳: " << initial_timestamp << " s" << std::endl;
    }
    if (normalize_position) {
        std::cout << "  位置已归一化，初始位置: [" 
                  << initial_position.x() << ", " 
                  << initial_position.y() << ", " 
                  << initial_position.z() << "] m" << std::endl;
    }
    
    return !gt_data_map_.empty();
}

bool GroundTruthReader::loadGenericGroundTruth(const std::string &file_path,
                                               const std::string &format) {
    clear();
    
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        GroundTruthData gt;
        std::istringstream iss(line);
        
        // 根据格式解析
        if (format.find("timestamp") != std::string::npos) {
            iss >> gt.timestamp;
        }
        
        if (format.find("x") != std::string::npos) {
            iss >> gt.x;
        }
        if (format.find("y") != std::string::npos) {
            iss >> gt.y;
        }
        if (format.find("z") != std::string::npos) {
            iss >> gt.z;
        }
        
        if (format.find("qx") != std::string::npos) {
            iss >> gt.qx >> gt.qy >> gt.qz >> gt.qw;
        }
        
        if (format.find("vx") != std::string::npos) {
            iss >> gt.vx >> gt.vy >> gt.vz;
        }
        
        if (gt.timestamp > 0) {
            gt_data_map_[gt.timestamp] = gt;
        }
    }
    
    file.close();
    
    std::cout << "成功加载 " << gt_data_map_.size() << " 条通用格式真值数据" << std::endl;
    return !gt_data_map_.empty();
}

bool GroundTruthReader::getHeightAtTime(double timestamp, double &height, 
                                       double max_time_diff) const {
    double nearest_ts = findNearestTimestamp(timestamp, max_time_diff);
    if (nearest_ts < 0) {
        return false;
    }
    
    auto it = gt_data_map_.find(nearest_ts);
    if (it != gt_data_map_.end()) {
        height = it->second.z;
        return true;
    }
    
    return false;
}

bool GroundTruthReader::getPositionAtTime(double timestamp, Eigen::Vector3d &position,
                                          double max_time_diff) const {
    double nearest_ts = findNearestTimestamp(timestamp, max_time_diff);
    if (nearest_ts < 0) {
        return false;
    }
    
    auto it = gt_data_map_.find(nearest_ts);
    if (it != gt_data_map_.end()) {
        position << it->second.x, it->second.y, it->second.z;
        return true;
    }
    
    return false;
}

bool GroundTruthReader::getGroundTruthAtTime(double timestamp, GroundTruthData &gt_data,
                                            double max_time_diff) const {
    double nearest_ts = findNearestTimestamp(timestamp, max_time_diff);
    if (nearest_ts < 0) {
        return false;
    }
    
    auto it = gt_data_map_.find(nearest_ts);
    if (it != gt_data_map_.end()) {
        gt_data = it->second;
        return true;
    }
    
    return false;
}

void GroundTruthReader::clear() {
    gt_data_map_.clear();
}

double GroundTruthReader::findNearestTimestamp(double timestamp, double max_time_diff) const {
    if (gt_data_map_.empty()) {
        return -1.0;
    }
    
    // 使用 lower_bound 查找
    auto it = gt_data_map_.lower_bound(timestamp);
    
    double best_ts = -1.0;
    double best_diff = max_time_diff;
    
    // 检查前一个元素
    if (it != gt_data_map_.begin()) {
        auto prev_it = std::prev(it);
        double diff = std::abs(prev_it->first - timestamp);
        if (diff < best_diff) {
            best_diff = diff;
            best_ts = prev_it->first;
        }
    }
    
    // 检查当前元素
    if (it != gt_data_map_.end()) {
        double diff = std::abs(it->first - timestamp);
        if (diff < best_diff) {
            best_diff = diff;
            best_ts = it->first;
        }
    }
    
    return best_ts;
}
