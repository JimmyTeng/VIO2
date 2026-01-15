/*******************************************************
 * EuRoC 数据集解析器
 * 
 * 简化版本：只保留设置路径和pop数据接口
 *******************************************************/

#include "init_eskf/euroc_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>

namespace init_eskf {

EuroCParser::EuroCParser() : initialized_(false) {
}

EuroCParser::~EuroCParser() {
}

bool EuroCParser::setDatasetPath(const std::string &dataset_path) {
    dataset_path_ = dataset_path;
    initialized_ = false;
    
    // 清空队列
    while (!data_queue_.empty()) {
        data_queue_.pop();
    }
    
    // 初始化数据队列
    return initializeDataQueue();
}

std::optional<DataItemResult> EuroCParser::popNext() {
    if (data_queue_.empty()) {
        return std::nullopt;
    }
    
    DataItem item = data_queue_.top();
    data_queue_.pop();
    
    DataItemResult result;
    result.type = item.type;
    
    switch (item.type) {
        case DataType::IMAGE:
            result.data = item.image;
            break;
        case DataType::IMU:
            result.data = item.imu;
            break;
        case DataType::GROUND_TRUTH:
            result.data = item.gt;
            break;
    }
    
    return result;
}

bool EuroCParser::initializeDataQueue() {
    if (dataset_path_.empty()) {
        std::cerr << "数据集路径未设置" << std::endl;
        return false;
    }
    
    // 构建文件路径
    std::string cam0_file = dataset_path_ + "/mav0/cam0/data.csv";
    std::string cam1_file = dataset_path_ + "/mav0/cam1/data.csv";
    std::string imu_file = dataset_path_ + "/mav0/imu0/data.csv";
    std::string gt_file = dataset_path_ + "/mav0/state_groundtruth_estimate0/data.csv";
    
    // 先加载图像数据并合并为双目数据
    bool image_success = loadStereoImageData(cam0_file, cam1_file);
    
    // 加载其他数据
    bool success = true;
    
    if (!image_success) {
        std::cerr << "警告: 无法加载图像数据" << std::endl;
        success = false;
    }
    
    if (!loadIMUData(imu_file)) {
        std::cerr << "警告: 无法加载 IMU 数据" << std::endl;
        success = false;
    }
    
    if (!loadGroundTruthData(gt_file)) {
        std::cerr << "警告: 无法加载真值数据" << std::endl;
        success = false;
    }
    
    if (success) {
        initialized_ = true;
        std::cout << "数据队列初始化完成，共 " << data_queue_.size() << " 条数据" << std::endl;
    }
    
    return success;
}

bool EuroCParser::loadStereoImageData(const std::string &cam0_file, const std::string &cam1_file) {
    // 使用map存储按时间戳索引的图像数据
    std::map<int64_t, ImageData> image_map;
    
    // 加载cam0数据
    std::ifstream file0(cam0_file);
    if (!file0.is_open()) {
        return false;
    }
    
    std::string line;
    std::getline(file0, line);  // 跳过标题行
    
    size_t cam0_count = 0;
    while (std::getline(file0, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;
        
        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }
        
        if (fields.size() >= 2) {
            int64_t timestamp = std::stoll(fields[0]);  // 纳秒
            ImageData img;
            img.timestamp = timestamp;
            img.filename0 = fields[1];
            
            image_map[timestamp] = img;
            cam0_count++;
        }
    }
    file0.close();
    
    // 加载cam1数据并合并
    std::ifstream file1(cam1_file);
    if (!file1.is_open()) {
        // 如果cam1不存在，只使用cam0数据
        std::cout << "警告: 无法打开 cam1 文件，仅使用 cam0 数据" << std::endl;
    } else {
        std::getline(file1, line);  // 跳过标题行
        
        size_t cam1_count = 0;
        size_t stereo_count = 0;
        const int64_t time_tolerance = 1000000;  // 1ms = 1000000 ns 时间容差
        
        while (std::getline(file1, line)) {
            if (line.empty()) continue;
            
            std::istringstream iss(line);
            std::string field;
            std::vector<std::string> fields;
            
            while (std::getline(iss, field, ',')) {
                fields.push_back(field);
            }
            
            if (fields.size() >= 2) {
                int64_t timestamp = std::stoll(fields[0]);  // 纳秒
                cam1_count++;
                
                // 查找匹配的时间戳（允许小的时间差）
                auto it = image_map.lower_bound(timestamp > time_tolerance ? timestamp - time_tolerance : 0);
                if (it != image_map.end()) {
                    int64_t diff = (it->first > timestamp) ? (it->first - timestamp) : (timestamp - it->first);
                    if (diff < time_tolerance) {
                        // 找到匹配的时间戳，合并为双目数据
                        it->second.filename1 = fields[1];
                        stereo_count++;
                    } else {
                        // 没有匹配的cam0数据，创建新的cam1数据项
                        ImageData img;
                        img.timestamp = timestamp;
                        img.filename1 = fields[1];
                        image_map[timestamp] = img;
                    }
                } else {
                    // 没有匹配的cam0数据，创建新的cam1数据项
                    ImageData img;
                    img.timestamp = timestamp;
                    img.filename1 = fields[1];
                    image_map[timestamp] = img;
                }
            }
        }
        file1.close();
        
        std::cout << "加载图像数据: cam0=" << cam0_count 
                  << ", cam1=" << cam1_count 
                  << ", 双目匹配=" << stereo_count << " 条" << std::endl;
    }
    
    // 将所有图像数据推入队列
    for (const auto& pair : image_map) {
        DataItem item;
        item.type = DataType::IMAGE;
        item.timestamp = pair.first;
        item.image = pair.second;
        data_queue_.push(item);
    }
    
    return true;
}

bool EuroCParser::loadIMUData(const std::string &csv_file) {
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    // 跳过标题行
    std::getline(file, line);
    
    size_t count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;
        
        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }
        
        // EuRoC IMU 格式: timestamp, wx, wy, wz, ax, ay, az
        // 注意：EuRoC原始坐标系是URF (Up-Right-Forward)
        //   URF: X=Up, Y=Right, Z=Forward
        // 需要转换为FLU (Forward-Left-Up)坐标系
        //   FLU: X=Forward, Y=Left, Z=Up
        // 转换关系：
        //   FLU_X = URF_Z (Forward)
        //   FLU_Y = -URF_Y (Left = -Right)
        //   FLU_Z = URF_X (Up)
        if (fields.size() >= 7) {
            DataItem item;
            item.type = DataType::IMU;
            item.timestamp = std::stoll(fields[0]);  // 纳秒
            item.imu.timestamp = item.timestamp;
            
            // 读取原始URF坐标系数据
            double urf_wx = std::stod(fields[1]);  // URF X (Up)
            double urf_wy = std::stod(fields[2]);  // URF Y (Right)
            double urf_wz = std::stod(fields[3]);  // URF Z (Forward)
            double urf_ax = std::stod(fields[4]);  // URF X (Up)
            double urf_ay = std::stod(fields[5]);  // URF Y (Right)
            double urf_az = std::stod(fields[6]);  // URF Z (Forward)
            
            // 转换为FLU坐标系
            // 角速度转换
            item.imu.gyr.x() = urf_wz;   // FLU X = URF Z (Forward)
            item.imu.gyr.y() = -urf_wy;  // FLU Y = -URF Y (Left = -Right)
            item.imu.gyr.z() = urf_wx;   // FLU Z = URF X (Up)
            
            // 加速度转换
            item.imu.acc.x() = urf_az;   // FLU X = URF Z (Forward)
            item.imu.acc.y() = -urf_ay;  // FLU Y = -URF Y (Left = -Right)
            item.imu.acc.z() = urf_ax;   // FLU Z = URF X (Up)
            
            // 验证：转换后，静止平放时加速度应该接近 [0, 0, 9.81]
            // 如果原始数据是 [9.81, 0, 0] (URF)，转换后应该是 [0, 0, 9.81] (FLU)
            
            data_queue_.push(item);
            count++;
        }
    }
    
    file.close();
    std::cout << "加载 IMU 数据: " << count << " 条" << std::endl;
    return true;
}

bool EuroCParser::loadGroundTruthData(const std::string &csv_file) {
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    // 跳过标题行
    std::getline(file, line);
    
    size_t count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string field;
        std::vector<std::string> fields;
        
        while (std::getline(iss, field, ',')) {
            fields.push_back(field);
        }
        
        // EuRoC 格式: timestamp, px, py, pz, qw, qx, qy, qz, vx, vy, vz, ...
        // 注意：真值数据使用FLU (Forward-Left-Up)坐标系（World坐标系）
        //   FLU: X=Forward, Y=Left, Z=Up
        // 真值数据已经是FLU坐标系，无需转换
        // 位置(x,y,z)和速度(vx,vy,vz)都在World坐标系中，使用FLU约定
        // 四元数(qw,qx,qy,qz)表示从World到Body的旋转
        if (fields.size() >= 11) {
            DataItem item;
            item.type = DataType::GROUND_TRUTH;
            item.timestamp = std::stoll(fields[0]);  // 纳秒
            item.gt.timestamp = item.timestamp;
            // 位置：World坐标系，FLU约定（无需转换）
            item.gt.x = std::stod(fields[1]);  // FLU X (Forward)
            item.gt.y = std::stod(fields[2]);  // FLU Y (Left)
            item.gt.z = std::stod(fields[3]);  // FLU Z (Up)
            // 四元数：从World到Body的旋转（Body坐标系是FLU）
            item.gt.qw = std::stod(fields[4]);
            item.gt.qx = std::stod(fields[5]);
            item.gt.qy = std::stod(fields[6]);
            item.gt.qz = std::stod(fields[7]);
            // 速度：World坐标系，FLU约定（无需转换）
            item.gt.vx = std::stod(fields[8]);  // FLU X (Forward)
            item.gt.vy = std::stod(fields[9]);  // FLU Y (Left)
            item.gt.vz = std::stod(fields[10]); // FLU Z (Up)
            
            data_queue_.push(item);
            count++;
        }
    }
    
    file.close();
    std::cout << "加载真值数据: " << count << " 条" << std::endl;
    return true;
}

} // namespace init_eskf
