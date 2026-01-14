/*******************************************************
 * EuRoC 数据集解析器
 * 
 * 简化版本：只保留设置路径和pop数据接口
 *******************************************************/

#pragma once

#include <string>
#include <queue>
#include <cstdint>
#include <optional>
#include <variant>
#include <Eigen/Dense>

namespace init_eskf {

/**
 * @brief 图像数据结构（支持双目）
 */
struct ImageData {
    int64_t timestamp;         // 时间戳 (ns)
    std::string filename0;      // 左相机（cam0）图像文件名
    std::string filename1;      // 右相机（cam1）图像文件名
    
    ImageData() : timestamp(0) {}
    
    /**
     * @brief 获取指定相机的文件名
     * @param camera_id 相机ID (0或1)
     * @return 对应相机的文件名，如果不存在返回空字符串
     */
    std::string getFilename(int camera_id) const {
        if (camera_id == 0) return filename0;
        if (camera_id == 1) return filename1;
        return "";
    }
    
    /**
     * @brief 检查指定相机是否有数据
     * @param camera_id 相机ID (0或1)
     * @return true 如果该相机有数据
     */
    bool hasCamera(int camera_id) const {
        if (camera_id == 0) return !filename0.empty();
        if (camera_id == 1) return !filename1.empty();
        return false;
    }
};

/**
 * @brief IMU 数据结构
 */
struct IMUData {
    int64_t timestamp;         // 时间戳 (ns)
    Eigen::Vector3d acc;        // 加速度 (m/s^2)
    Eigen::Vector3d gyr;        // 角速度 (rad/s)
    
    IMUData() : timestamp(0), 
                acc(Eigen::Vector3d::Zero()),
                gyr(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief 真值数据结构
 */
struct GroundTruthData {
    int64_t timestamp;    // 时间戳 (ns)
    double x, y, z;       // 位置 (m)
    double qx, qy, qz, qw; // 四元数
    double vx, vy, vz;    // 速度 (m/s)

    GroundTruthData() : timestamp(0), x(0), y(0), z(0),
                       qx(0), qy(0), qz(0), qw(1),
                       vx(0), vy(0), vz(0) {}
};

/**
 * @brief 数据类型枚举
 */
enum class DataType {
    IMAGE,
    IMU,
    GROUND_TRUTH
};

/**
 * @brief 数据变体类型，可以存储不同类型的数据
 */
using DataVariant = std::variant<ImageData, IMUData, GroundTruthData>;

/**
 * @brief 数据项结构，包含类型和数据
 */
struct DataItemResult {
    DataType type;
    DataVariant data;
    
    /**
     * @brief 获取时间戳
     */
    int64_t getTimestamp() const {
        switch (type) {
            case DataType::IMAGE:
                return std::get<ImageData>(data).timestamp;
            case DataType::IMU:
                return std::get<IMUData>(data).timestamp;
            case DataType::GROUND_TRUTH:
                return std::get<GroundTruthData>(data).timestamp;
        }
        return 0;
    }
    
    /**
     * @brief 获取图像数据（如果是图像类型）
     */
    const ImageData& getImage() const {
        return std::get<ImageData>(data);
    }
    
    /**
     * @brief 获取IMU数据（如果是IMU类型）
     */
    const IMUData& getIMU() const {
        return std::get<IMUData>(data);
    }
    
    /**
     * @brief 获取真值数据（如果是真值类型）
     */
    const GroundTruthData& getGroundTruth() const {
        return std::get<GroundTruthData>(data);
    }
};

/**
 * @brief EuRoC 数据集解析器
 */
class EuroCParser {
public:
    EuroCParser();
    ~EuroCParser();
    
    /**
     * @brief 设置数据集路径
     * @param dataset_path 数据集根目录路径（例如：/path/to/MH_01_easy）
     * @return true 如果路径设置成功
     */
    bool setDatasetPath(const std::string &dataset_path);
    
    /**
     * @brief 弹出下一个数据（按时间顺序，不管什么类型）
     * @return optional<DataItemResult> 如果成功弹出数据，返回包含类型和数据的结构；如果没有更多数据，返回 std::nullopt
     */
    std::optional<DataItemResult> popNext();
    
private:
    /**
     * @brief 内部数据结构，用于按时间排序
     */
    struct DataItem {
        DataType type;
        int64_t timestamp;  // 时间戳 (ns)
        ImageData image;
        IMUData imu;
        GroundTruthData gt;
        
        bool operator<(const DataItem& other) const {
            return timestamp > other.timestamp;  // 最小堆
        }
    };
    
    /**
     * @brief 初始化数据队列
     */
    bool initializeDataQueue();
    
    /**
     * @brief 加载双目图像数据到队列（合并cam0和cam1）
     */
    bool loadStereoImageData(const std::string &cam0_file, const std::string &cam1_file);
    
    /**
     * @brief 加载IMU数据到队列
     */
    bool loadIMUData(const std::string &csv_file);
    
    /**
     * @brief 加载真值数据到队列
     */
    bool loadGroundTruthData(const std::string &csv_file);
    
    // 数据集路径
    std::string dataset_path_;
    
    // 数据队列（按时间排序）
    std::priority_queue<DataItem> data_queue_;
    
    // 是否已初始化
    bool initialized_;
};

} // namespace init_eskf