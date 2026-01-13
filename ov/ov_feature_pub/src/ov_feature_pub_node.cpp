#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// OpenVINS includes
// Note: ov_core exports headers from src/ directory, so we include without ov_core prefix
#include <track/TrackKLT.h>
#include <cam/CamRadtan.h>
#include <cam/CamEqui.h>
#include <utils/sensor_data.h>
#include <feat/Feature.h>
#include <feat/FeatureDatabase.h>

#include <Eigen/Dense>
#include <memory>
#include <unordered_map>

using namespace std;
using namespace ov_core;

// Global variables
ros::Publisher pub_img, pub_match;
ros::Publisher pub_restart;

// Tracker instance
std::shared_ptr<TrackKLT> tracker;
std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras;

// State variables
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
double prev_image_time = 0;
bool init_pub = false;

// Parameters
std::string IMAGE_TOPIC;
std::string CAMERA_CALIB_FILE;
int MAX_CNT = 150;
int MIN_DIST = 30;
int FREQ = 10;
int SHOW_TRACK = 0;
int NUM_OF_CAM = 1;        // Default: monocular camera (cam0)
bool USE_STEREO = false;   // Default: single camera mode
bool EQUALIZE = false;
int FAST_THRESHOLD = 20;
int GRID_X = 5;
int GRID_Y = 5;
int MIN_PX_DIST = 10;
std::string DIST_MODEL = "radtan"; // "radtan" or "equidistant"

// Previous feature positions for velocity calculation
std::unordered_map<size_t, cv::Point2f> prev_pts_map;

/**
 * @brief Load camera calibration from VINS-Mono config YAML file
 */
Eigen::MatrixXd loadCameraCalibFromConfig(cv::FileStorage& fsSettings, int& width, int& height) {
    Eigen::MatrixXd calib = Eigen::MatrixXd::Zero(8, 1);
    
    // Read image dimensions (VINS-Mono format)
    width = (int)fsSettings["image_width"];
    height = (int)fsSettings["image_height"];
    
    // Read projection parameters (VINS-Mono format: fx, fy, cx, cy)
    cv::FileNode proj_node = fsSettings["projection_parameters"];
    if (!proj_node.empty()) {
        std::vector<double> proj_params;
        proj_node >> proj_params;
        if (proj_params.size() >= 4) {
            calib(0) = proj_params[0];  // fx
            calib(1) = proj_params[1];  // fy
            calib(2) = proj_params[2];  // cx
            calib(3) = proj_params[3];  // cy
        }
    }
    
    // Read distortion parameters (VINS-Mono format: k1, k2, p1, p2)
    cv::FileNode dist_node = fsSettings["distortion_parameters"];
    if (!dist_node.empty()) {
        std::vector<double> dist_params;
        dist_node >> dist_params;
        if (dist_params.size() >= 4) {
            calib(4) = dist_params[0];  // k1
            calib(5) = dist_params[1];  // k2
            calib(6) = dist_params[2];  // p1
            calib(7) = dist_params[3];  // p2
        }
    }
    
    // Read distortion model (VINS-Mono uses PINHOLE with radial-tangential)
    std::string model_type;
    fsSettings["model_type"] >> model_type;
    if (model_type == "PINHOLE") {
        DIST_MODEL = "radtan";  // PINHOLE model uses radial-tangential distortion
    } else if (model_type == "FISHEYE") {
        DIST_MODEL = "equidistant";
    }
    
    return calib;
}

/**
 * @brief Initialize tracker with camera calibration from config file
 */
bool initializeTracker(cv::FileStorage& fsSettings) {
    int img_width = 752, img_height = 480;  // Default EuRoC resolution
    Eigen::MatrixXd cam_calib = loadCameraCalibFromConfig(fsSettings, img_width, img_height);
    
    ROS_INFO("Camera calibration loaded: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
             cam_calib(0), cam_calib(1), cam_calib(2), cam_calib(3));
    ROS_INFO("Image resolution: %dx%d", img_width, img_height);
    ROS_INFO("Distortion model: %s", DIST_MODEL.c_str());
    
    // Create camera object
    std::shared_ptr<CamBase> camera;
    if (DIST_MODEL == "equidistant" || DIST_MODEL == "equi") {
        camera = std::make_shared<CamEqui>(img_width, img_height);
    } else {
        camera = std::make_shared<CamRadtan>(img_width, img_height);
    }
    camera->set_value(cam_calib);
    
    cameras[0] = camera;
    
    // Create tracker
    TrackBase::HistogramMethod hist_method = EQUALIZE ? 
        TrackBase::HistogramMethod::HISTOGRAM : TrackBase::HistogramMethod::NONE;
    
    tracker = std::make_shared<TrackKLT>(
        cameras, MAX_CNT, 0, USE_STEREO, hist_method,
        FAST_THRESHOLD, GRID_X, GRID_Y, MIN_PX_DIST
    );
    
    ROS_INFO("Tracker initialized successfully");
    return true;
}

/**
 * @brief Image callback function
 */
void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    ROS_DEBUG("Received image: stamp=%.6f, width=%d, height=%d", 
              img_msg->header.stamp.toSec(), img_msg->width, img_msg->height);
    
    if (first_image_flag) {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        prev_image_time = img_msg->header.stamp.toSec();
        ROS_INFO("First image received! Starting feature tracking...");
        // Don't return, process the first image to initialize tracker
    }
    
    // Detect unstable camera stream
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || 
        img_msg->header.stamp.toSec() < last_image_time) {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        prev_image_time = 0;
        pub_count = 1;
        prev_pts_map.clear();
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    
    // Frequency control
    bool PUB_THIS_FRAME = false;
    double time_elapsed = img_msg->header.stamp.toSec() - first_image_time;
    if (time_elapsed > 0.001) {  // Avoid division by zero
        if (round(1.0 * pub_count / time_elapsed) <= FREQ) {
            PUB_THIS_FRAME = true;
            if (abs(1.0 * pub_count / time_elapsed - FREQ) < 0.01 * FREQ) {
                first_image_time = img_msg->header.stamp.toSec();
                pub_count = 0;
            }
        }
    } else {
        // First few frames, always publish
        PUB_THIS_FRAME = true;
    }
    
    // Convert ROS image to OpenCV
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    
    cv::Mat img = ptr->image;
    
    // Prepare CameraData for OpenVINS tracker (monocular mode: cam0 only)
    CameraData message;
    message.timestamp = img_msg->header.stamp.toSec();
    message.sensor_ids.push_back(0);  // cam0 (monocular)
    message.images.push_back(img);
    message.masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));
    
    // Feed image to tracker
    try {
        tracker->feed_new_camera(message);
    } catch (const std::exception& e) {
        ROS_ERROR("Error in tracker->feed_new_camera: %s", e.what());
        return;
    }
    
    // Get tracked features
    auto pts_last = tracker->get_last_obs();
    auto ids_last = tracker->get_last_ids();
    
    ROS_DEBUG("Tracked features: %zu points", pts_last[0].size());
    
    if (PUB_THIS_FRAME && !pts_last[0].empty()) {
        pub_count++;
        
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;
        
        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";
        
        const auto& cur_pts = pts_last[0];
        const auto& cur_ids = ids_last[0];
        
        // Get feature database to check track lengths
        auto database = tracker->get_feature_database();
        
        for (size_t j = 0; j < cur_ids.size() && j < cur_pts.size(); j++) {
            size_t feat_id = cur_ids[j];
            cv::Point2f cur_pt = cur_pts[j].pt;
            
            // Get normalized coordinates (undistorted)
            cv::Point2f un_pt = cameras[0]->undistort_cv(cur_pt);
            
            // Calculate velocity from previous position
            float vx = 0.0, vy = 0.0;
            if (prev_pts_map.find(feat_id) != prev_pts_map.end() && prev_image_time > 0) {
                cv::Point2f prev_pt = prev_pts_map[feat_id];
                double dt = img_msg->header.stamp.toSec() - prev_image_time;
                if (dt > 0 && dt < 1.0) {  // Reasonable time interval
                    vx = (cur_pt.x - prev_pt.x) / dt;
                    vy = (cur_pt.y - prev_pt.y) / dt;
                }
            }
            
            // Only publish features that have been tracked for at least 2 frames
            // Check track length from database
            bool should_publish = false;
            if (database) {
                Feature feat;
                if (database->get_feature_clone(feat_id, feat)) {
                    // Check if feature has been tracked for at least 2 frames
                    if (!feat.uvs[0].empty() && feat.uvs[0].size() >= 2) {
                        should_publish = true;
                    }
                } else {
                    // New feature, but we'll include it if it has a previous position
                    should_publish = (prev_pts_map.find(feat_id) != prev_pts_map.end());
                }
            } else {
                // If no database access, use simple heuristic
                should_publish = (prev_pts_map.find(feat_id) != prev_pts_map.end());
            }
            
            if (should_publish) {
                geometry_msgs::Point32 p;
                p.x = un_pt.x;
                p.y = un_pt.y;
                p.z = 1;
                
                feature_points->points.push_back(p);
                id_of_point.values.push_back(feat_id * NUM_OF_CAM + 0);
                u_of_point.values.push_back(cur_pt.x);
                v_of_point.values.push_back(cur_pt.y);
                velocity_x_of_point.values.push_back(vx);
                velocity_y_of_point.values.push_back(vy);
            }
            
            // Update previous position
            prev_pts_map[feat_id] = cur_pt;
        }
        
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        
        ROS_INFO("Publishing %lu features at time %.6f", 
                 feature_points->points.size(),
                 feature_points->header.stamp.toSec());
        
        // Skip the first image; since no optical speed on first image
        if (!init_pub) {
            init_pub = true;
        } else {
            pub_img.publish(feature_points);
            
            // Visualize tracked features if requested (similar to VINS-Mono)
            if (SHOW_TRACK) {
                cv::Mat show_img;
                cv::cvtColor(img, show_img, CV_GRAY2BGR);
                
                // Draw all tracked features (not just published ones)
                for (size_t j = 0; j < cur_ids.size() && j < cur_pts.size(); j++) {
                    cv::Point2f pt = cur_pts[j].pt;
                    
                    // Get track length for color coding
                    int track_length = 1;
                    if (database) {
                        Feature feat;
                        if (database->get_feature_clone(cur_ids[j], feat)) {
                            track_length = feat.uvs[0].size();
                        }
                    }
                    
                    // Color based on track length (similar to VINS-Mono)
                    // Blue = new features, Red = long tracks
                    double len = std::min(1.0, 1.0 * track_length / 20.0);
                    cv::Scalar color(255 * (1 - len), 0, 255 * len);
                    cv::circle(show_img, pt, 2, color, 2);
                }
                
                sensor_msgs::ImagePtr img_msg_out = cv_bridge::CvImage(
                    img_msg->header, "bgr8", show_img).toImageMsg();
                pub_match.publish(img_msg_out);
            }
        }
        
        // Update previous image time for velocity calculation
        prev_image_time = img_msg->header.stamp.toSec();
    }
}

/**
 * @brief Read parameters from ROS parameter server
 */
void readParameters(ros::NodeHandle &n) {
    // Read config file path
    std::string config_file;
    n.param<std::string>("config_file", config_file, "");
    
    if (config_file.empty()) {
        ROS_ERROR("config_file parameter not set!");
        ros::shutdown();
        return;
    }
    
    // Read parameters from config file
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        ROS_ERROR("ERROR: Wrong path to settings file: %s", config_file.c_str());
        ros::shutdown();
        return;
    }
    
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    
    // Try to read camera_calib_file (optional, for separate calib file)
    // If not found, we'll use parameters directly from config file (VINS-Mono style)
    cv::FileNode calib_file_node = fsSettings["camera_calib_file"];
    if (!calib_file_node.empty()) {
        calib_file_node >> CAMERA_CALIB_FILE;
    }
    
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    FREQ = fsSettings["freq"];
    SHOW_TRACK = fsSettings["show_track"];
    
    // OpenVINS specific parameters
    n.param<int>("max_features", MAX_CNT, MAX_CNT);
    n.param<int>("fast_threshold", FAST_THRESHOLD, 20);
    n.param<int>("grid_x", GRID_X, 5);
    n.param<int>("grid_y", GRID_Y, 5);
    n.param<int>("min_px_dist", MIN_PX_DIST, 10);
    n.param<bool>("equalize", EQUALIZE, false);
    n.param<std::string>("distortion_model", DIST_MODEL, "radtan");
    
    ROS_INFO("Parameters loaded:");
    ROS_INFO("  image_topic: %s", IMAGE_TOPIC.c_str());
    ROS_INFO("  max_features: %d", MAX_CNT);
    ROS_INFO("  freq: %d", FREQ);
    ROS_INFO("  camera_mode: MONOCULAR (cam0, NUM_OF_CAM=%d, USE_STEREO=%s)", 
             NUM_OF_CAM, USE_STEREO ? "true" : "false");
    
    // Initialize tracker with camera calibration from config file
    if (!initializeTracker(fsSettings)) {
        ROS_ERROR("Failed to initialize tracker!");
        fsSettings.release();
        ros::shutdown();
        return;
    }
    
    fsSettings.release();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ov_feature_pub");
    ros::NodeHandle n("~");
    ros::NodeHandle nh;  // Public node handle for subscribing to absolute topics
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    readParameters(n);
    
    // Tracker is initialized in readParameters() function
    
    // Setup subscribers and publishers
    // Use public node handle for subscribing to absolute topic paths
    ros::Subscriber sub_img = nh.subscribe(IMAGE_TOPIC, 100, img_callback);
    
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img", 1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);
    
    ROS_INFO("OpenVINS Feature Publisher started (MONOCULAR mode)");
    ROS_INFO("Subscribing to: %s", IMAGE_TOPIC.c_str());
    ROS_INFO("Publishing to: /feature_tracker/feature");
    ROS_INFO("Publishing debug image to: /feature_tracker/feature_img (SHOW_TRACK=%d)", SHOW_TRACK);
    ROS_INFO("Camera: cam0 (single camera)");
    
    // Wait a moment for topics to be advertised
    ros::Duration(0.5).sleep();
    
    // Check if topic exists
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    bool topic_found = false;
    for (const auto& topic : master_topics) {
        if (topic.name == IMAGE_TOPIC) {
            topic_found = true;
            ROS_INFO("Topic %s found! Waiting for messages...", IMAGE_TOPIC.c_str());
            break;
        }
    }
    if (!topic_found) {
        ROS_WARN("Topic %s not found! Make sure the bag file is playing or camera is publishing.", IMAGE_TOPIC.c_str());
        ROS_WARN("Available image topics:");
        for (const auto& topic : master_topics) {
            if (topic.datatype == "sensor_msgs/Image") {
                ROS_WARN("  - %s", topic.name.c_str());
            }
        }
    }
    
    ros::spin();
    return 0;
}
