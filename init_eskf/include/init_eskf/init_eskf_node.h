/*******************************************************
 * Init-ESKF Node: 独立的初始化模块 ROS 节点
 * 
 * 这是一个独立的 ROS 节点，用于运行 Init-ESKF 初始化系统
 * 可以订阅 IMU 数据，从数据包读取真值高度，运行初始化，并发布结果
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include "init_eskf/init_eskf.h"
#include "init_eskf/gt_reader.h"
#include <queue>
#include <mutex>
#include <thread>

/**
 * @brief Init-ESKF ROS 节点类
 * 
 * 功能：
 * - 订阅 IMU 数据
 * - 从数据包读取真值高度（可选）
 * - 运行 Init-ESKF 初始化
 * - 发布初始化状态和结果
 */
class InitESKFNode {
public:
    InitESKFNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~InitESKFNode();
    
    /**
     * @brief 初始化节点
     * @return true 如果初始化成功
     */
    bool initialize();
    
    /**
     * @brief 运行节点主循环
     */
    void run();
    
private:
    // ROS 相关
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // 订阅者
    ros::Subscriber sub_imu_;
    
    // 发布者
    ros::Publisher pub_odometry_;
    ros::Publisher pub_path_;
    ros::Publisher pub_converged_;
    ros::Publisher pub_state_info_;
    
    // TF 广播器
    tf::TransformBroadcaster tf_broadcaster_;
    
    // Init-ESKF 实例
    init_eskf::InitESKF eskf_;
    
    // 真值数据读取器
    GroundTruthReader gt_reader_;
    
    // 数据缓冲区
    std::queue<sensor_msgs::ImuConstPtr> imu_buffer_;
    std::mutex buffer_mutex_;
    
    // 参数
    std::string imu_topic_;
    std::string frame_id_;
    std::string child_frame_id_;
    
    // 真值数据相关
    bool use_gt_height_;
    std::string gt_type_;  // "kitti", "euroc", "generic"
    std::string gt_path_;
    std::string gt_format_;
    double max_time_diff_;
    
    // Init-ESKF 参数
    double gravity_mag_;
    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;
    
    // 量测参数
    double tof_noise_;
    double flow_noise_;
    
    // 收敛阈值
    double convergence_pos_threshold_;
    double convergence_vel_threshold_;
    double convergence_ori_threshold_;
    
    // 状态
    bool initialized_;
    bool converged_;
    nav_msgs::Path path_;
    
    // 重力对齐相关
    bool use_gravity_alignment_;
    double gravity_alignment_duration_;
    double max_acc_variance_for_alignment_;
    std::vector<std::pair<double, Eigen::Vector3d>> acc_samples_for_alignment_;
    std::vector<std::pair<double, Eigen::Vector3d>> gyr_samples_for_alignment_;
    bool gravity_alignment_done_;
    
    // 光流相关（如果需要）
    bool use_optical_flow_;
    Eigen::Vector2d last_flow_velocity_;
    double last_flow_time_;
    
    // 回调函数
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
    
    // 处理函数
    void processIMU(const sensor_msgs::ImuConstPtr& imu_msg);
    void processTOF(double timestamp);
    void processFlow(double timestamp);
    void checkConvergence();
    void publishResults();
    
    // 辅助函数
    void loadParameters();
    bool loadGroundTruth();
    void publishOdometry(const init_eskf::InitESKF::NominalState& state, double timestamp);
    void publishPath(const init_eskf::InitESKF::NominalState& state, double timestamp);
    void publishTF(const init_eskf::InitESKF::NominalState& state, double timestamp);
};
