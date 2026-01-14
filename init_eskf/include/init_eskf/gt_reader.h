/*******************************************************
 * Ground Truth Reader: 从数据包读取真值高度
 * 
 * 支持多种数据格式：
 * - KITTI 格式 (oxts/data/目录下的txt文件)
 * - EuRoC 格式 (groundtruth.csv)
 * - 通用格式 (timestamp, x, y, z, ...)
 *******************************************************/

#pragma once

#include <string>
#include <map>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief 真值数据结构
 */
struct GroundTruthData {
    double timestamp;      // 时间戳 (s)
    double x, y, z;        // 位置 (m)
    double qx, qy, qz, qw; // 四元数
    double vx, vy, vz;     // 速度 (m/s)
    
    GroundTruthData() : timestamp(0), x(0), y(0), z(0),
                       qx(0), qy(0), qz(0), qw(1),
                       vx(0), vy(0), vz(0) {}
};

/**
 * @brief 真值数据读取器
 */
class GroundTruthReader {
public:
    GroundTruthReader();
    ~GroundTruthReader();
    
    /**
     * @brief 从 KITTI 格式文件加载真值数据
     * @param data_folder KITTI 数据文件夹路径
     * @param timestamps_file 时间戳文件路径
     * @return true 如果加载成功
     */
    bool loadKITTIGroundTruth(const std::string &data_folder, 
                             const std::string &timestamps_file = "");
    
    /**
     * @brief 从 EuRoC 格式 CSV 文件加载真值数据
     * @param csv_file CSV 文件路径
     * @return true 如果加载成功
     */
    bool loadEuRoCGroundTruth(const std::string &csv_file);
    
    /**
     * @brief 从 EuRoC 格式 CSV 文件加载真值数据，并将初始点设置为 0
     * @param csv_file CSV 文件路径
     * @param normalize_time 是否将第一个时间戳归一化为 0（默认 true）
     * @param normalize_position 是否将第一个位置归一化为 (0,0,0)（默认 true）
     * @return true 如果加载成功
     */
    bool loadEuRoCGroundTruthWithOrigin(const std::string &csv_file,
                                       bool normalize_time = true,
                                       bool normalize_position = true);
    
    /**
     * @brief 从通用格式文件加载真值数据
     * @param file_path 文件路径
     * @param format 格式字符串: "timestamp x y z" 或 "timestamp x y z qx qy qz qw"
     * @return true 如果加载成功
     */
    bool loadGenericGroundTruth(const std::string &file_path, 
                               const std::string &format = "timestamp x y z");
    
    /**
     * @brief 获取指定时间戳的真值高度
     * @param timestamp 时间戳 (s)
     * @param height 输出的高度值 (m)
     * @param max_time_diff 最大时间差 (s)，用于查找最近的真值
     * @return true 如果找到对应的真值
     */
    bool getHeightAtTime(double timestamp, double &height, double max_time_diff = 0.1) const;
    
    /**
     * @brief 获取指定时间戳的真值位置
     * @param timestamp 时间戳 (s)
     * @param position 输出的位置 (x, y, z)
     * @param max_time_diff 最大时间差 (s)
     * @return true 如果找到对应的真值
     */
    bool getPositionAtTime(double timestamp, Eigen::Vector3d &position, 
                          double max_time_diff = 0.1) const;
    
    /**
     * @brief 获取指定时间戳的完整真值数据
     * @param timestamp 时间戳 (s)
     * @param gt_data 输出的真值数据
     * @param max_time_diff 最大时间差 (s)
     * @return true 如果找到对应的真值
     */
    bool getGroundTruthAtTime(double timestamp, GroundTruthData &gt_data,
                             double max_time_diff = 0.1) const;
    
    /**
     * @brief 检查是否已加载数据
     */
    bool isLoaded() const { return !gt_data_map_.empty(); }
    
    /**
     * @brief 获取数据数量
     */
    size_t getDataCount() const { return gt_data_map_.size(); }
    
    /**
     * @brief 清空已加载的数据
     */
    void clear();
    
private:
    // 时间戳到真值数据的映射
    std::map<double, GroundTruthData> gt_data_map_;
    
    // 辅助函数：查找最近的时间戳
    double findNearestTimestamp(double timestamp, double max_time_diff) const;
};
