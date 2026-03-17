#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>

class TableCalibratorNode : public rclcpp::Node {
public:
    TableCalibratorNode() : Node("table_calibrator_node"), calibrated_(false) {
        
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 3D Physical Table Corners (Center of net is 0,0,0)
        table_corners_3d_[0] = cv::Point3f(-1.37f,  0.7625f, 0.0f); // Top-Left
        table_corners_3d_[1] = cv::Point3f( 1.37f,  0.7625f, 0.0f); // Top-Right
        table_corners_3d_[2] = cv::Point3f(-1.37f, -0.7625f, 0.0f); // Bottom-Left
        table_corners_3d_[3] = cv::Point3f( 1.37f, -0.7625f, 0.0f); // Bottom-Right

        // Modern OpenCV 4.7+ Setup
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        
        // Ensure parameters are initialized properly
        cv::aruco::DetectorParameters params;
        aruco_params_ = params;
        
        aruco_detector_ = cv::aruco::ArucoDetector(aruco_dict_, aruco_params_);

        cv::namedWindow("ArUco Calibration Debug", cv::WINDOW_AUTOSIZE);

        RCLCPP_INFO(this->get_logger(), "Auto-Calibrator Online. Look at the debug window!");

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/left/camera_info", 10, std::bind(&TableCalibratorNode::infoCallback, this, std::placeholders::_1));
            
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/left/image_raw", 10, std::bind(&TableCalibratorNode::imageCallback, this, std::placeholders::_1));
    }

    ~TableCalibratorNode() {
        cv::destroyAllWindows();
    }

private:
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (got_camera_info_) return;
        camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void*)msg->d.data()).clone();
        got_camera_info_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!got_camera_info_) return;

        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat debug_frame = frame.clone();

            if (calibrated_) {
                cv::putText(debug_frame, "CALIBRATION COMPLETE!", cv::Point(50, 50), 
                            cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
                cv::imshow("ArUco Calibration Debug", debug_frame);
                cv::waitKey(1);
                return; 
            }

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

            // Detect the markers
            aruco_detector_.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

            if (!markerIds.empty()) {
                cv::aruco::drawDetectedMarkers(debug_frame, markerCorners, markerIds);
            }

            std::string status = "Markers found: " + std::to_string(markerIds.size()) + "/4";
            cv::putText(debug_frame, status, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

            cv::imshow("ArUco Calibration Debug", debug_frame);
            cv::waitKey(1);

            if (markerIds.size() >= 4) {
                std::vector<cv::Point2f> image_points;
                std::vector<cv::Point3f> object_points;
                
                int valid_corners = 0;

                for (size_t i = 0; i < markerIds.size(); i++) {
                    int id = markerIds[i];
                    if (table_corners_3d_.find(id) != table_corners_3d_.end()) {
                        cv::Point2f center(0.f, 0.f);
                        for (auto& pt : markerCorners[i]) { center += pt; }
                        center.x /= 4.f; center.y /= 4.f;

                        image_points.push_back(center);
                        object_points.push_back(table_corners_3d_[id]);
                        valid_corners++;
                    }
                }

                if (valid_corners == 4) {
                    cv::Mat rvec, tvec;
                    cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);

                    publishStaticTransform(rvec, tvec);
                    
                    calibrated_ = true;
                    RCLCPP_INFO(this->get_logger(), "SUCCESS! Table calibrated. Generating Transform.");
                }
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void publishStaticTransform(const cv::Mat& rvec, const cv::Mat& tvec) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "table"; 
        t.child_frame_id = "camera_left_optical_frame";

        t.transform.translation.x = tvec.at<double>(0);
        t.transform.translation.y = tvec.at<double>(1);
        t.transform.translation.z = tvec.at<double>(2);

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        tf2::Matrix3x3 tf2_rot(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
        );
        tf2::Quaternion q;
        tf2_rot.getRotation(q);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    bool calibrated_;
    bool got_camera_info_ = false;
    cv::Mat camera_matrix_, dist_coeffs_;
    std::map<int, cv::Point3f> table_corners_3d_;
    
    // THE FIX: These are the exact modern types matching the code at the top.
    cv::aruco::Dictionary aruco_dict_;
    cv::aruco::DetectorParameters aruco_params_;
    cv::aruco::ArucoDetector aruco_detector_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TableCalibratorNode>());
    rclcpp::shutdown();
    return 0;
}
