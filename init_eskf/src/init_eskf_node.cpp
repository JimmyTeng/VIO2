/*******************************************************
 * Init-ESKF Node: 独立的初始化模块 ROS 节点
 *******************************************************/

#include "init_eskf/init_eskf_node.h"
#include <ros/package.h>
#include <algorithm>

using namespace init_eskf;

InitESKFNode::InitESKFNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private),
      use_gt_height_(true), use_optical_flow_(false),
      use_gravity_alignment_(true),
      gravity_alignment_duration_(2.0),
      max_acc_variance_for_alignment_(0.1),
      gravity_alignment_done_(false),
      last_flow_time_(0.0),
      initialized_(false), converged_(false) {
    
    // 加载参数
    loadParameters();
    
    // 初始化发布者
    pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("/init_eskf/odometry", 10);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/init_eskf/path", 10);
    pub_converged_ = nh_.advertise<std_msgs::Bool>("/init_eskf/converged", 10);
    pub_state_info_ = nh_.advertise<std_msgs::Float64>("/init_eskf/uncertainty", 10);
    
    // 初始化订阅者
    sub_imu_ = nh_.subscribe(imu_topic_, 2000, &InitESKFNode::imuCallback, this);
    
    // 初始化路径
    path_.header.frame_id = frame_id_;
}

InitESKFNode::~InitESKFNode() {}

bool InitESKFNode::initialize() {
    // 加载真值数据
    if (use_gt_height_) {
        if (!loadGroundTruth()) {
            ROS_WARN("Failed to load ground truth data, continuing without it...");
            use_gt_height_ = false;
        }
    }
    
    // 如果启用重力对齐，等待收集足够的 IMU 数据
    if (use_gravity_alignment_) {
        ROS_INFO("Waiting for gravity alignment data (%.1f seconds)...", 
                 gravity_alignment_duration_);
        // 重力对齐将在 processIMU 中完成
        initialized_ = false;  // 等待重力对齐完成
    } else {
        // 直接初始化 ESKF（使用 Identity 姿态，不推荐）
        eskf_.initialize(gravity_mag_, acc_noise_, gyr_noise_, 
                       acc_bias_noise_, gyr_bias_noise_);
        initialized_ = true;
        ROS_WARN("Gravity alignment is disabled. This may cause issues if the system starts on a slope.");
    }
    
    ROS_INFO("Init-ESKF Node initialized successfully");
    
    return true;
}

void InitESKFNode::run() {
    ros::Rate rate(200);  // 200Hz 处理频率
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // 处理 IMU 数据
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            while (!imu_buffer_.empty()) {
                sensor_msgs::ImuConstPtr imu_msg = imu_buffer_.front();
                imu_buffer_.pop();
                processIMU(imu_msg);
            }
        }
        
        // 检查收敛并发布结果
        if (initialized_ && !converged_) {
            checkConvergence();
            publishResults();
        }
        
        rate.sleep();
    }
}

void InitESKFNode::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.push(imu_msg);
}

void InitESKFNode::processIMU(const sensor_msgs::ImuConstPtr& imu_msg) {
    double timestamp = imu_msg->header.stamp.toSec();
    
    // 提取 IMU 数据
    Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                       imu_msg->linear_acceleration.y,
                       imu_msg->linear_acceleration.z);
    Eigen::Vector3d gyr(imu_msg->angular_velocity.x,
                       imu_msg->angular_velocity.y,
                       imu_msg->angular_velocity.z);
    
    // 如果启用重力对齐且尚未完成，收集样本
    if (use_gravity_alignment_ && !gravity_alignment_done_) {
        acc_samples_for_alignment_.push_back({timestamp, acc});
        gyr_samples_for_alignment_.push_back({timestamp, gyr});
        
        // 保持样本窗口大小（只保留最近 N 秒的数据）
        double cutoff_time = timestamp - gravity_alignment_duration_ - 1.0;
        acc_samples_for_alignment_.erase(
            std::remove_if(acc_samples_for_alignment_.begin(), 
                          acc_samples_for_alignment_.end(),
                          [cutoff_time](const std::pair<double, Eigen::Vector3d> &p) {
                              return p.first < cutoff_time;
                          }),
            acc_samples_for_alignment_.end());
        gyr_samples_for_alignment_.erase(
            std::remove_if(gyr_samples_for_alignment_.begin(), 
                          gyr_samples_for_alignment_.end(),
                          [cutoff_time](const std::pair<double, Eigen::Vector3d> &p) {
                              return p.first < cutoff_time;
                          }),
            gyr_samples_for_alignment_.end());
        
        // 检查是否有足够的数据进行重力对齐
        if (acc_samples_for_alignment_.size() > 100) {  // 至少 100 个样本（约 0.5 秒 @ 200Hz）
            double oldest_time = acc_samples_for_alignment_.front().first;
            double time_span = timestamp - oldest_time;
            
            if (time_span >= gravity_alignment_duration_) {
                // 尝试重力对齐
                if (eskf_.initializeWithGravityAlignment(
                        acc_samples_for_alignment_,
                        gyr_samples_for_alignment_,
                        gravity_alignment_duration_,
                        max_acc_variance_for_alignment_)) {
                    gravity_alignment_done_ = true;
                    initialized_ = true;
                    ROS_INFO("==========================================");
                    ROS_INFO("Gravity Alignment Completed!");
                    ROS_INFO("==========================================");
                    init_eskf::InitESKF::NominalState state = eskf_.getState();
                    ROS_INFO("Initial orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]", 
                             state.q.w(), state.q.x(), state.q.y(), state.q.z());
                    ROS_INFO("Initial acc bias: [%.6f, %.6f, %.6f] m/s^2", 
                             state.ba.x(), state.ba.y(), state.ba.z());
                    ROS_INFO("Initial gyr bias: [%.6f, %.6f, %.6f] rad/s", 
                             state.bg.x(), state.bg.y(), state.bg.z());
                } else {
                    ROS_DEBUG("Gravity alignment failed, waiting for more data...");
                }
            }
        }
        
        // 如果重力对齐未完成，不进行传播
        if (!gravity_alignment_done_) {
            return;
        }
    }
    
    if (!initialized_) return;
    
    // IMU 传播
    eskf_.propagate(timestamp, acc, gyr);
    
    // TOF 更新（50Hz，从真值读取）
    static double last_tof_time = 0.0;
    double tof_period = 0.02;  // 50Hz
    if (timestamp - last_tof_time >= tof_period) {
        processTOF(timestamp);
        last_tof_time = timestamp;
    }
    
    // 光流更新（50Hz，如果需要）
    if (use_optical_flow_) {
        static double last_flow_update_time = 0.0;
        double flow_period = 0.02;  // 50Hz
        if (timestamp - last_flow_update_time >= flow_period) {
            processFlow(timestamp);
            last_flow_update_time = timestamp;
        }
    }
}

void InitESKFNode::processTOF(double timestamp) {
    if (!use_gt_height_ || !gt_reader_.isLoaded()) {
        return;
    }
    
    double height = 0.0;
    if (gt_reader_.getHeightAtTime(timestamp, height, max_time_diff_)) {
        eskf_.updateTOF(timestamp, height, tof_noise_);
        ROS_DEBUG("TOF update: t=%.3f, height=%.3f m", timestamp, height);
    }
}

void InitESKFNode::processFlow(double timestamp) {
    // 这里可以从光流传感器话题获取数据
    // 目前使用模拟数据或从其他话题订阅
    // TODO: 实现光流数据订阅
    
    // 示例：使用模拟光流速度
    Eigen::Vector2d flow_vel(0.0, 0.0);  // 需要从实际传感器获取
    eskf_.updateFlow(timestamp, flow_vel, flow_noise_);
}

void InitESKFNode::checkConvergence() {
    if (converged_) return;
    
    bool converged = eskf_.checkConvergence(
        convergence_pos_threshold_,
        convergence_vel_threshold_,
        convergence_ori_threshold_
    );
    
    if (converged && !converged_) {
        converged_ = true;
        ROS_INFO("==========================================");
        ROS_INFO("Init-ESKF State Converged!");
        ROS_INFO("==========================================");
        
        init_eskf::InitESKF::NominalState state = eskf_.getState();
        ROS_INFO("Position: [%.3f, %.3f, %.3f] m", 
                 state.p.x(), state.p.y(), state.p.z());
        ROS_INFO("Velocity: [%.3f, %.3f, %.3f] m/s", 
                 state.v.x(), state.v.y(), state.v.z());
        ROS_INFO("Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]", 
                 state.q.w(), state.q.x(), state.q.y(), state.q.z());
        ROS_INFO("Accelerometer bias: [%.6f, %.6f, %.6f] m/s^2", 
                 state.ba.x(), state.ba.y(), state.ba.z());
        ROS_INFO("Gyroscope bias: [%.6f, %.6f, %.6f] rad/s", 
                 state.bg.x(), state.bg.y(), state.bg.z());
    }
}

void InitESKFNode::publishResults() {
    init_eskf::InitESKF::NominalState state = eskf_.getState();
    double timestamp = eskf_.getTimestamp();
    
    // 发布里程计
    publishOdometry(state, timestamp);
    
    // 发布路径
    publishPath(state, timestamp);
    
    // 发布 TF
    publishTF(state, timestamp);
    
    // 发布收敛状态
    std_msgs::Bool converged_msg;
    converged_msg.data = converged_;
    pub_converged_.publish(converged_msg);
    
    // 发布不确定性信息
    init_eskf::Matrix15d P = eskf_.getCovariance();
    double pos_uncertainty = sqrt(P.block<3, 3>(0, 0).trace() / 3.0);
    std_msgs::Float64 uncertainty_msg;
    uncertainty_msg.data = pos_uncertainty;
    pub_state_info_.publish(uncertainty_msg);
}

void InitESKFNode::publishOdometry(const init_eskf::InitESKF::NominalState& state, double timestamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(timestamp);
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    
    // 位置
    odom.pose.pose.position.x = state.p.x();
    odom.pose.pose.position.y = state.p.y();
    odom.pose.pose.position.z = state.p.z();
    
    // 姿态
    odom.pose.pose.orientation.w = state.q.w();
    odom.pose.pose.orientation.x = state.q.x();
    odom.pose.pose.orientation.y = state.q.y();
    odom.pose.pose.orientation.z = state.q.z();
    
    // 速度
    odom.twist.twist.linear.x = state.v.x();
    odom.twist.twist.linear.y = state.v.y();
    odom.twist.twist.linear.z = state.v.z();
    
    // 协方差（从 ESKF 获取）
    init_eskf::Matrix15d P = eskf_.getCovariance();
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (i < 3 && j < 3) {
                odom.pose.covariance[i * 6 + j] = P(i, j);
            } else if (i >= 3 && j >= 3 && i < 6 && j < 6) {
                odom.twist.covariance[(i - 3) * 6 + (j - 3)] = P(i + 3, j + 3);
            }
        }
    }
    
    pub_odometry_.publish(odom);
}

void InitESKFNode::publishPath(const init_eskf::InitESKF::NominalState& state, double timestamp) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(timestamp);
    pose_stamped.header.frame_id = frame_id_;
    
    pose_stamped.pose.position.x = state.p.x();
    pose_stamped.pose.position.y = state.p.y();
    pose_stamped.pose.position.z = state.p.z();
    
    pose_stamped.pose.orientation.w = state.q.w();
    pose_stamped.pose.orientation.x = state.q.x();
    pose_stamped.pose.orientation.y = state.q.y();
    pose_stamped.pose.orientation.z = state.q.z();
    
    path_.poses.push_back(pose_stamped);
    path_.header.stamp = ros::Time(timestamp);
    
    pub_path_.publish(path_);
}

void InitESKFNode::publishTF(const init_eskf::InitESKF::NominalState& state, double timestamp) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.p.x(), state.p.y(), state.p.z()));
    tf::Quaternion q(state.q.x(), state.q.y(), state.q.z(), state.q.w());
    transform.setRotation(q);
    
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform, ros::Time(timestamp), frame_id_, child_frame_id_)
    );
}

void InitESKFNode::loadParameters() {
    // IMU 话题
    nh_private_.param<std::string>("imu_topic", imu_topic_, "/imu0");
    
    // 坐标系
    nh_private_.param<std::string>("frame_id", frame_id_, "world");
    nh_private_.param<std::string>("child_frame_id", child_frame_id_, "imu");
    
    // 真值数据相关
    nh_private_.param<bool>("use_gt_height", use_gt_height_, true);
    nh_private_.param<std::string>("gt_type", gt_type_, "euroc");
    nh_private_.param<std::string>("gt_path", gt_path_, "");
    nh_private_.param<std::string>("gt_format", gt_format_, "timestamp x y z");
    nh_private_.param<double>("max_time_diff", max_time_diff_, 0.1);
    
    // Init-ESKF 参数
    nh_private_.param<double>("gravity_mag", gravity_mag_, 9.81);
    nh_private_.param<double>("acc_noise", acc_noise_, 0.01);
    nh_private_.param<double>("gyr_noise", gyr_noise_, 0.001);
    nh_private_.param<double>("acc_bias_noise", acc_bias_noise_, 1e-5);
    nh_private_.param<double>("gyr_bias_noise", gyr_bias_noise_, 1e-6);
    
    // 量测参数
    nh_private_.param<double>("tof_noise", tof_noise_, 0.05);
    nh_private_.param<double>("flow_noise", flow_noise_, 0.1);
    
    // 收敛阈值
    nh_private_.param<double>("convergence_pos_threshold", convergence_pos_threshold_, 0.1);
    nh_private_.param<double>("convergence_vel_threshold", convergence_vel_threshold_, 0.1);
    nh_private_.param<double>("convergence_ori_threshold", convergence_ori_threshold_, 0.05);
    
    // 重力对齐
    nh_private_.param<bool>("use_gravity_alignment", use_gravity_alignment_, true);
    nh_private_.param<double>("gravity_alignment_duration", gravity_alignment_duration_, 2.0);
    nh_private_.param<double>("max_acc_variance_for_alignment", max_acc_variance_for_alignment_, 0.1);
    
    // 光流
    nh_private_.param<bool>("use_optical_flow", use_optical_flow_, false);
    
    ROS_INFO("Init-ESKF Node Parameters:");
    ROS_INFO("  IMU topic: %s", imu_topic_.c_str());
    ROS_INFO("  Frame ID: %s", frame_id_.c_str());
    ROS_INFO("  Child Frame ID: %s", child_frame_id_.c_str());
    ROS_INFO("  Use GT height: %s", use_gt_height_ ? "true" : "false");
    ROS_INFO("  GT type: %s", gt_type_.c_str());
    ROS_INFO("  GT path: %s", gt_path_.c_str());
    ROS_INFO("  Use gravity alignment: %s", use_gravity_alignment_ ? "true" : "false");
    if (use_gravity_alignment_) {
        ROS_INFO("  Gravity alignment duration: %.1f s", gravity_alignment_duration_);
        ROS_INFO("  Max acc variance for alignment: %.3f", max_acc_variance_for_alignment_);
    }
}

bool InitESKFNode::loadGroundTruth() {
    if (gt_path_.empty()) {
        ROS_WARN("Ground truth path is empty, skipping ground truth loading");
        return false;
    }
    
    bool success = false;
    if (gt_type_ == "kitti") {
        success = gt_reader_.loadKITTIGroundTruth(gt_path_);
    } else if (gt_type_ == "euroc") {
        success = gt_reader_.loadEuRoCGroundTruth(gt_path_);
    } else if (gt_type_ == "generic") {
        success = gt_reader_.loadGenericGroundTruth(gt_path_, gt_format_);
    } else {
        ROS_ERROR("Unknown ground truth type: %s", gt_type_.c_str());
        return false;
    }
    
    if (success) {
        ROS_INFO("Successfully loaded %zu ground truth data points", 
                 gt_reader_.getDataCount());
    } else {
        ROS_ERROR("Failed to load ground truth data from: %s", gt_path_.c_str());
    }
    
    return success;
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "init_eskf_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    InitESKFNode node(nh, nh_private);
    
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize Init-ESKF Node");
        return -1;
    }
    
    ROS_INFO("Init-ESKF Node started");
    node.run();
    
    return 0;
}
