#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>

class ArucoCalibrateCamera : public rclcpp::Node
{
public:
    ArucoCalibrateCamera() : Node("aruco_calibrate_camera")
    {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("marker_size", 0.10);
        this->declare_parameter("auto_calibrate", false);
        
        camera_id_ = this->get_parameter("camera_id").as_string();
        marker_size_ = this->get_parameter("marker_size").as_double();
        auto_calibrate_ = this->get_parameter("auto_calibrate").as_bool();
        
        // ArUco setup - OpenCV 4.10.0 syntax
        dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));
        detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();
        
        // Camera intrinsics (defaults)
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            320.0, 0.0, 320.0,
            0.0, 320.0, 200.0,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        
        // Table marker positions (standard table tennis table)
        marker_positions_[0] = cv::Point3f(-0.7625, -1.37, 0.0);   // Front-left
        marker_positions_[1] = cv::Point3f(0.7625, -1.37, 0.0);    // Front-right
        marker_positions_[2] = cv::Point3f(0.7625, 1.37, 0.0);     // Back-right
        marker_positions_[3] = cv::Point3f(-0.7625, 1.37, 0.0);    // Back-left
        
        // Subscribe to camera
        std::string image_topic = "/camera/" + camera_id_ + "/image_raw";
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&ArucoCalibrateCamera::imageCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "ArUco calibration for camera '%s'", camera_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Place 10cm ArUco markers (IDs 0-3) at table corners");
        if (auto_calibrate_) {
            RCLCPP_INFO(this->get_logger(), "Auto-calibrate mode: will calibrate automatically");
        } else {
            RCLCPP_INFO(this->get_logger(), "Press 'c' to calibrate | 's' to save");
        }
    }

private:
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
        cv::Mat display;
        cv::cvtColor(frame, display, cv::COLOR_GRAY2BGR);
        
        // Detect markers - OpenCV 4.10.0 syntax
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);
        
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(display, corners, ids);
            
            // Check which markers we have
            detected_count_ = 0;
            for (int id : ids) {
                if (id >= 0 && id <= 3) {
                    detected_count_++;
                }
            }
            
            std::string status = "Detected " + std::to_string(detected_count_) + "/4 markers";
            if (detected_count_ >= 3) {
                status += " - Ready";
                cv::putText(display, status, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                
                // Auto-calibrate if enabled and not done yet
                if (auto_calibrate_ && !calibrated_) {
                    current_ids_ = ids;
                    current_corners_ = corners;
                    calibrate();
                }
            } else {
                status += " - Need 3+";
                cv::putText(display, status, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
            }
            
            // Store for manual calibration
            current_ids_ = ids;
            current_corners_ = corners;
        }
        
        if (calibrated_) {
            cv::putText(display, "CALIBRATED - Press 's' to save", cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("ArUco Calibration - " + camera_id_, display);
        int key = cv::waitKey(1);
        
        if (key == 'c' || key == 'C') {
            if (detected_count_ >= 3) {
                calibrate();
            } else {
                RCLCPP_WARN(this->get_logger(), "Need at least 3 markers!");
            }
        }
        
        if (key == 's' || key == 'S') {
            if (calibrated_) {
                saveToYAML();
            } else {
                RCLCPP_WARN(this->get_logger(), "Run calibration first (press 'c')");
            }
        }
    }
    
    void calibrate()
    {
        RCLCPP_INFO(this->get_logger(), "Calibrating with %d markers...", detected_count_);
        
        // Build correspondences
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;
        
        for (size_t i = 0; i < current_ids_.size(); i++) {
            int id = current_ids_[i];
            if (marker_positions_.find(id) != marker_positions_.end()) {
                object_points.push_back(marker_positions_[id]);
                
                // Use marker center
                cv::Point2f center(0, 0);
                for (const auto& pt : current_corners_[i]) {
                    center += pt;
                }
                center *= 0.25f;
                image_points.push_back(center);
            }
        }
        
        // Solve PnP
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(object_points, image_points,
                                     camera_matrix_, dist_coeffs_,
                                     rvec, tvec);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Calibration failed!");
            return;
        }
        
        // Convert to camera position in table frame
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat camera_pos = -R.t() * tvec;
        
        x_ = camera_pos.at<double>(0);
        y_ = camera_pos.at<double>(1);
        z_ = camera_pos.at<double>(2);
        
        // Convert rotation to RPY
        cv::Mat R_inv = R.t();
        roll_ = atan2(R_inv.at<double>(2, 1), R_inv.at<double>(2, 2));
        pitch_ = atan2(-R_inv.at<double>(2, 0), 
                       sqrt(R_inv.at<double>(2, 1) * R_inv.at<double>(2, 1) + 
                            R_inv.at<double>(2, 2) * R_inv.at<double>(2, 2)));
        yaw_ = atan2(R_inv.at<double>(1, 0), R_inv.at<double>(0, 0));
        
        calibrated_ = true;
        
        RCLCPP_INFO(this->get_logger(), "=== CALIBRATION RESULT ===");
        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f (meters)", x_, y_, z_);
        RCLCPP_INFO(this->get_logger(), "Rotation: roll=%.3f, pitch=%.3f, yaw=%.3f (radians)", 
                   roll_, pitch_, yaw_);
        RCLCPP_INFO(this->get_logger(), "         roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                   roll_ * 180 / M_PI, pitch_ * 180 / M_PI, yaw_ * 180 / M_PI);
        
        if (auto_calibrate_) {
            saveToYAML();
        } else {
            RCLCPP_INFO(this->get_logger(), "Press 's' to save");
        }
    }
    
    void saveToYAML()
    {
        std::string filename = "/tmp/camera_" + camera_id_ + "_calibration.yaml";
        std::ofstream file(filename);
        
        file << "camera_id: " << camera_id_ << "\n";
        file << "calibration_method: aruco_markers\n";
        file << "markers_detected: " << detected_count_ << "\n\n";
        
        file << "camera_pose:\n";
        file << "  position:\n";
        file << "    x: " << x_ << "\n";
        file << "    y: " << y_ << "\n";
        file << "    z: " << z_ << "\n";
        file << "  orientation_radians:\n";
        file << "    roll: " << roll_ << "\n";
        file << "    pitch: " << pitch_ << "\n";
        file << "    yaw: " << yaw_ << "\n";
        file << "  orientation_degrees:\n";
        file << "    roll: " << (roll_ * 180 / M_PI) << "\n";
        file << "    pitch: " << (pitch_ * 180 / M_PI) << "\n";
        file << "    yaw: " << (yaw_ * 180 / M_PI) << "\n";
        
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "=========================");
        RCLCPP_INFO(this->get_logger(), "Saved to: %s", filename.c_str());
        RCLCPP_INFO(this->get_logger(), "Copy these values into:");
        RCLCPP_INFO(this->get_logger(), "  src/ttt_calibration/config/stereo_extrinsic.yaml");
        RCLCPP_INFO(this->get_logger(), "=========================");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    std::string camera_id_;
    double marker_size_;
    bool auto_calibrate_;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;
    
    std::map<int, cv::Point3f> marker_positions_;
    
    std::vector<int> current_ids_;
    std::vector<std::vector<cv::Point2f>> current_corners_;
    int detected_count_ = 0;
    
    bool calibrated_ = false;
    double x_, y_, z_, roll_, pitch_, yaw_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoCalibrateCamera>());
    rclcpp::shutdown();
    return 0;
}
