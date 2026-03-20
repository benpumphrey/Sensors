#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class ArucoTableCalibrationNode : public rclcpp::Node
{
public:
    ArucoTableCalibrationNode() : Node("aruco_table_calibration")
    {
        // Parameters
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("markers.dictionary_id", 10);
        this->declare_parameter("markers.marker_size", 0.10);
        this->declare_parameter("show_detection", true);
        this->declare_parameter("auto_calibrate", false);
        
        camera_id_ = this->get_parameter("camera_id").as_string();
        int dict_id = this->get_parameter("markers.dictionary_id").as_int();
        marker_size_ = this->get_parameter("markers.marker_size").as_double();
        show_detection_ = this->get_parameter("show_detection").as_bool();
        auto_calibrate_ = this->get_parameter("auto_calibrate").as_bool();
        
        // Load marker positions from config
        loadMarkerPositions();
        
        // Load camera intrinsics
        loadCameraIntrinsics();
        
        // ArUco setup
        dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
        detector_params_ = cv::aruco::DetectorParameters::create();
        
        // Subscribe to camera
        std::string image_topic = "/camera/" + camera_id_ + "/image_raw";
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&ArucoTableCalibrationNode::imageCallback, this, std::placeholders::_1)
        );
        
        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "ArUco table calibration started");
        RCLCPP_INFO(this->get_logger(), "Camera: %s", camera_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Marker size: %.2f m", marker_size_);
        RCLCPP_INFO(this->get_logger(), "Press 'c' to calibrate and save transform");
    }

private:
    void loadMarkerPositions()
    {
        // Load from parameter server
        // Marker positions in table frame (meters)
        marker_positions_3d_[0] = cv::Point3f(-0.7625, -1.37, 0.0);   // Front-left
        marker_positions_3d_[1] = cv::Point3f(0.7625, -1.37, 0.0);    // Front-right
        marker_positions_3d_[2] = cv::Point3f(0.7625, 1.37, 0.0);     // Back-right
        marker_positions_3d_[3] = cv::Point3f(-0.7625, 1.37, 0.0);    // Back-left
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu marker positions", marker_positions_3d_.size());
    }
    
    void loadCameraIntrinsics()
    {
        // Load from camera_intrinsic.yaml (defaults for now)
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            500.0, 0.0, 320.0,
            0.0, 500.0, 200.0,
            0.0, 0.0, 1.0);
        
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
        
        RCLCPP_INFO(this->get_logger(), "Loaded camera intrinsics (default values)");
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat frame = cv_ptr->image;
        cv::Mat display_frame;
        cv::cvtColor(frame, display_frame, cv::COLOR_GRAY2BGR);
        
        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, detector_params_);
        
        if (!marker_ids.empty()) {
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(display_frame, marker_corners, marker_ids);
            
            // Estimate pose for each marker
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, 
                                                 camera_matrix_, dist_coeffs_, 
                                                 rvecs, tvecs);
            
            // Draw axes for each marker
            for (size_t i = 0; i < marker_ids.size(); i++) {
                cv::aruco::drawAxis(display_frame, camera_matrix_, dist_coeffs_,
                                   rvecs[i], tvecs[i], marker_size_ * 0.5);
                
                int id = marker_ids[i];
                detected_markers_[id] = std::make_pair(rvecs[i], tvecs[i]);
                
                // Display marker info
                std::string label = "ID:" + std::to_string(id) + 
                                   " (" + std::to_string(detected_markers_.size()) + "/4)";
                cv::putText(display_frame, label,
                           marker_corners[i][0], cv::FONT_HERSHEY_SIMPLEX,
                           0.5, cv::Scalar(0, 255, 0), 2);
            }
            
            // Status display
            if (detected_markers_.size() >= 3) {
                std::string status = "Detected " + std::to_string(detected_markers_.size()) + 
                                    "/4 markers - Press 'c' to calibrate";
                cv::putText(display_frame, status, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                           
                if (auto_calibrate_ && !calibration_done_) {
                    performCalibration();
                    calibration_done_ = true;
                }
            } else {
                std::string status = "Detected " + std::to_string(detected_markers_.size()) + 
                                    "/4 markers - Need at least 3";
                cv::putText(display_frame, status, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
            }
        }
        
        // Show detection window
        if (show_detection_) {
            cv::imshow("ArUco Table Calibration - " + camera_id_, display_frame);
            int key = cv::waitKey(1);
            
            if (key == 'c' || key == 'C') {
                if (detected_markers_.size() >= 3) {
                    performCalibration();
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                               "Need at least 3 markers, only %zu detected", 
                               detected_markers_.size());
                }
            }
        }
    }
    
    void performCalibration()
    {
        RCLCPP_INFO(this->get_logger(), "Performing calibration with %zu markers...", 
                   detected_markers_.size());
        
        // Build point correspondences
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;
        
        for (const auto& [id, pose] : detected_markers_) {
            if (marker_positions_3d_.find(id) != marker_positions_3d_.end()) {
                object_points.push_back(marker_positions_3d_[id]);
                
                // Get marker center in image
                cv::Vec3d tvec = pose.second;
                cv::Point2f center(tvec[0], tvec[1]);
                image_points.push_back(center);
            }
        }
        
        if (object_points.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough valid markers for calibration");
            return;
        }
        
        // Solve PnP to get camera pose relative to table
        cv::Vec3d rvec, tvec;
        bool success = cv::solvePnP(object_points, image_points, 
                                     camera_matrix_, dist_coeffs_,
                                     rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "solvePnP failed");
            return;
        }
        
        // Convert rotation vector to matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        // Invert to get table-to-camera transform
        cv::Mat R_inv = rotation_matrix.t();
        cv::Mat t_inv = -R_inv * cv::Mat(tvec);
        
        // Extract position and orientation
        double x = t_inv.at<double>(0);
        double y = t_inv.at<double>(1);
        double z = t_inv.at<double>(2);
        
        // Convert rotation matrix to Euler angles
        tf2::Matrix3x3 tf_rotation(
            R_inv.at<double>(0, 0), R_inv.at<double>(0, 1), R_inv.at<double>(0, 2),
            R_inv.at<double>(1, 0), R_inv.at<double>(1, 1), R_inv.at<double>(1, 2),
            R_inv.at<double>(2, 0), R_inv.at<double>(2, 1), R_inv.at<double>(2, 2)
        );
        
        double roll, pitch, yaw;
        tf_rotation.getRPY(roll, pitch, yaw);
        
        RCLCPP_INFO(this->get_logger(), "Calibration complete!");
        RCLCPP_INFO(this->get_logger(), "Camera position: (%.3f, %.3f, %.3f) m", x, y, z);
        RCLCPP_INFO(this->get_logger(), "Camera orientation: roll=%.2f°, pitch=%.2f°, yaw=%.2f°",
                   roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        
        // Save to file
        saveCalibration(x, y, z, roll, pitch, yaw);
        
        // Publish TF transform
        publishTransform(x, y, z, roll, pitch, yaw);
    }
    
    void saveCalibration(double x, double y, double z, double roll, double pitch, double yaw)
    {
        std::string output_path = "/tmp/aruco_calibration_" + camera_id_ + ".yaml";
        
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "camera_id" << YAML::Value << camera_id_;
        out << YAML::Key << "calibration_method" << YAML::Value << "aruco_markers";
        out << YAML::Key << "markers_detected" << YAML::Value << detected_markers_.size();
        
        out << YAML::Key << "camera_pose";
        out << YAML::BeginMap;
        out << YAML::Key << "position";
        out << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << x;
        out << YAML::Key << "y" << YAML::Value << y;
        out << YAML::Key << "z" << YAML::Value << z;
        out << YAML::EndMap;
        
        out << YAML::Key << "orientation_radians";
        out << YAML::BeginMap;
        out << YAML::Key << "roll" << YAML::Value << roll;
        out << YAML::Key << "pitch" << YAML::Value << pitch;
        out << YAML::Key << "yaw" << YAML::Value << yaw;
        out << YAML::EndMap;
        
        out << YAML::Key << "orientation_degrees";
        out << YAML::BeginMap;
        out << YAML::Key << "roll" << YAML::Value << (roll * 180.0 / M_PI);
        out << YAML::Key << "pitch" << YAML::Value << (pitch * 180.0 / M_PI);
        out << YAML::Key << "yaw" << YAML::Value << (yaw * 180.0 / M_PI);
        out << YAML::EndMap;
        out << YAML::EndMap;
        out << YAML::EndMap;
        
        std::ofstream fout(output_path);
        fout << out.c_str();
        fout.close();
        
        RCLCPP_INFO(this->get_logger(), "Saved to: %s", output_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Update stereo_extrinsic.yaml with these values");
    }
    
    void publishTransform(double x, double y, double z, double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "table";
        transform.child_frame_id = "camera_" + camera_id_ + "_optical_frame";
        
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
        
        RCLCPP_INFO(this->get_logger(), "Published TF transform: table -> camera_%s_optical_frame",
                   camera_id_.c_str());
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    
    std::string camera_id_;
    double marker_size_;
    bool show_detection_;
    bool auto_calibrate_;
    bool calibration_done_ = false;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    std::map<int, cv::Point3f> marker_positions_3d_;  // Marker ID -> 3D position on table
    std::map<int, std::pair<cv::Vec3d, cv::Vec3d>> detected_markers_;  // ID -> (rvec, tvec)
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoTableCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}