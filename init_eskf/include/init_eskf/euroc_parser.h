/*******************************************************
 * EuRoC 数据集解析器
 * 
 * 专门用于解析 EuRoC 数据集，支持初始点归一化
 * 针对 V1_01_easy 数据集
 *******************************************************/

#pragma once

#include <string>
#include <vector>
#include <functional>
#include <Eigen/Dense>
#include "init_eskf/gt_reader.h"

namespace init_eskf {

/**
 * @brief IMU 数据结构
 */
struct IMUData {
    double timestamp;           // 时间戳 (s)
    Eigen::Vector3d acc;        // 加速度 (m/s^2)
    Eigen::Vector3d gyr;        // 角速度 (rad/s)
    
    IMUData() : timestamp(0.0), 
                acc(Eigen::Vector3d::Zero()),
                gyr(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief EuRoC 数据集解析器
 */
class EuroCParser {
public:
    EuroCParser();
    ~EuroCParser();
    
    /**
     * @brief 加载 EuRoC 数据集
     * @param dataset_path 数据集根目录路径（例如：/path/to/V1_01_easy）
     * @param normalize_time 是否将第一个时间戳归一化为 0（默认 true）
     * @param normalize_position 是否将第一个位置归一化为 (0,0,0)（默认 true）
     * @return true 如果加载成功
     */
    bool loadDataset(const std::string &dataset_path,
                    bool normalize_time = true,
                    bool normalize_position = true);
    
    /**
     * @brief 获取指定时间戳的真值数据
     * @param timestamp 时间戳 (s)
     * @param gt_data 输出的真值数据
     * @param max_time_diff 最大时间差 (s)
     * @return true 如果找到对应的真值
     */
    bool getGroundTruthAtTime(double timestamp, GroundTruthData &gt_data,
                             double max_time_diff = 0.01) const;
    
    /**
     * @brief 获取指定时间戳的 IMU 数据
     * @param timestamp 时间戳 (s)
     * @param imu_data 输出的 IMU 数据
     * @param max_time_diff 最大时间差 (s)
     * @return true 如果找到对应的 IMU 数据
     */
    bool getIMUAtTime(double timestamp, IMUData &imu_data,
                     double max_time_diff = 0.005) const;
    
    /**
     * @brief 获取初始时间戳（归一化前的原始值）
     */
    double getInitialTimestamp() const { return initial_timestamp_; }
    
    /**
     * @brief 获取初始位置（归一化前的原始值）
     */
    Eigen::Vector3d getInitialPosition() const { return initial_position_; }
    
    /**
     * @brief 获取真值数据数量
     */
    size_t getGroundTruthCount() const { return gt_data_.size(); }
    
    /**
     * @brief 获取 IMU 数据数量
     */
    size_t getIMUCount() const { return imu_data_.size(); }
    
    /**
     * @brief 获取所有真值数据（用于批量处理）
     */
    const std::vector<GroundTruthData>& getGroundTruthData() const { return gt_data_; }
    
    /**
     * @brief 获取所有 IMU 数据（用于批量处理）
     */
    const std::vector<IMUData>& getIMUData() const { return imu_data_; }

    /**
     * @brief 以流式方式扫描真值数据：一行一行读取并回调处理，不做整体存储
     * @param csv_file 真值 CSV 文件路径
     * @param handler  对每一条 GroundTruthData 的处理回调
     * @return true 如果读取过程中未发生错误
     *
     * 说明：该接口不修改内部状态（不写 gt_data_ / initial_timestamp_ 等），
     * 适合做快速检查 / 统计，而不是作为 loadDataset 的替代。
     */
    bool scanGroundTruth(
        const std::string &csv_file,
        const std::function<void(const GroundTruthData&)> &handler) const;

    /**
     * @brief 以流式方式扫描 IMU 数据：一行一行读取并回调处理，不做整体存储
     * @param csv_file IMU CSV 文件路径
     * @param handler  对每一条 IMUData 的处理回调
     * @return true 如果读取过程中未发生错误
     *
     * 说明：该接口同样不修改内部状态。
     */
    bool scanIMU(
        const std::string &csv_file,
        const std::function<void(const IMUData&)> &handler) const;
    
private:
    /**
     * @brief 加载真值数据
     * @param normalize_time_base 时间归一化的基准时间戳（如果 normalize_time=true）
     */
    bool loadGroundTruth(const std::string &csv_file,
                        bool normalize_time,
                        bool normalize_position,
                        double normalize_time_base = 0.0);
    
    /**
     * @brief 加载 IMU 数据
     * @param normalize_time_base 时间归一化的基准时间戳（如果 normalize_time=true）
     */
    bool loadIMUData(const std::string &csv_file,
                    bool normalize_time,
                    double normalize_time_base = 0.0);
    
    // 数据集路径
    std::string dataset_path_;
    
    // 初始值（归一化前的原始值）
    double initial_timestamp_;
    Eigen::Vector3d initial_position_;
    
    // 归一化标志
    bool time_normalized_;
    bool position_normalized_;
    
    // 数据存储
    std::vector<GroundTruthData> gt_data_;
    std::vector<IMUData> imu_data_;
};

} // namespace init_eskf
