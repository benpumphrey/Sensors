#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VisionNode : public rclcpp::Node
{
public:
    VisionNode() : Node("vision_node")
    {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("debug_mode", true);
        this->declare_parameter("blur_size", 5);
        this->declare_parameter("min_radius", 3);
        this->declare_parameter("min_brightness", 20);

        camera_id_ = this->get_parameter("camera_id").as_string();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        blur_size_ = this->get_parameter("blur_size").as_int();
        min_radius_ = this->get_parameter("min_radius").as_int();
        min_brightness_ = this->get_parameter("min_brightness").as_int();

        if (blur_size_ % 2 == 0) blur_size_++;

        std::string image_topic = "/camera/" + camera_id_ + "/image_raw";
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&VisionNode::imageCallback, this, std::placeholders::_1)
        );

        std::string detection_topic = "/ball_detection/" + camera_id_;
        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>(
            detection_topic, 10
        );

        std::string annotated_topic = "/camera/" + camera_id_ + "/image_annotated";
        annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            annotated_topic, 10
        );

        cv::namedWindow("Vision Tracker", cv::WINDOW_NORMAL);

        RCLCPP_INFO(this->get_logger(), "Vision Node running. No max radius limit.");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 1. FAST MEMORY ACCESS: Point to the existing ROS buffer instead of copying it.
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        cv::Mat small_frame;
        cv::resize(frame, small_frame, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    
        cv::Mat annotated_frame;
        cv::cvtColor(small_frame, annotated_frame, cv::COLOR_GRAY2BGR);

        cv::Mat blurred;
        cv::GaussianBlur(small_frame, blurred, cv::Size(blur_size_, blur_size_), 0, 0);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
            small_frame.rows / 8, 100, 20,
            min_radius_ / 2, 0); // min_radius scaled for downsampling

        auto detection_msg = ttt_msgs::msg::BallDetection();
        detection_msg.header = msg->header;

        int brightest_idx = -1;
        int max_brightness = 0;

        if (!circles.empty()) {
            for (size_t i = 0; i < circles.size(); i++) {
                cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);

                // ROI check on the small frame to find the ball's brightness
                int x = std::max(0, center.x - radius);
                int y = std::max(0, center.y - radius);
                int w = std::min(2 * radius, small_frame.cols - x);
                int h = std::min(2 * radius, small_frame.rows - y);

                if (w > 0 && h > 0) {
                    cv::Rect roi(x, y, w, h);
                    cv::Scalar avg = cv::mean(small_frame(roi));
                    if (avg[0] > max_brightness) {
                        max_brightness = avg[0];
                        brightest_idx = i;
                    }
                }
            }
        }

        if (brightest_idx != -1 && max_brightness >= min_brightness_) {
            float best_x_small = circles[brightest_idx][0];
            float best_y_small = circles[brightest_idx][1];
            float best_r_small = circles[brightest_idx][2];

            // Map back to the original 1280x720 resolution for other ROS nodes
            detection_msg.x = best_x_small * 2.0;
            detection_msg.y = best_y_small * 2.0;
            detection_msg.radius = best_r_small * 2.0;
            detection_msg.confidence = max_brightness / 255.0;

            // Draw the tracking box and current brightness value on the screen
            cv::Point top_left(best_x_small - best_r_small, best_y_small - best_r_small);
            cv::Point bottom_right(best_x_small + best_r_small, best_y_small + best_r_small);
            cv::rectangle(annotated_frame, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);

            std::string hud_text = "BRIGHT: " + std::to_string(max_brightness);
            cv::putText(annotated_frame, hud_text,
                cv::Point(best_x_small - best_r_small, best_y_small - best_r_small - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

            if (debug_mode_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "Tracking: X:%.1f Y:%.1f (Brightness: %d)",
                    detection_msg.x, detection_msg.y, max_brightness);
            }
        }
        else {
            // Clear detection if ball is lost or too dim
            detection_msg.x = -1.0;
            detection_msg.y = -1.0;
            detection_msg.radius = 0.0;
            detection_msg.confidence = 0.0;
        }

        // Send the 3D-ready 2D coordinates to the network
        detection_pub_->publish(detection_msg);

        if (debug_mode_) {
            // Auto-create window if it doesn't exist
            if (cv::getWindowProperty("Vision Tracker", cv::WND_PROP_VISIBLE) < 1) {
                cv::namedWindow("Vision Tracker", cv::WINDOW_NORMAL);
            }
            cv::imshow("Vision Tracker", annotated_frame);
            cv::waitKey(1);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

    std::string camera_id_;
    bool debug_mode_;
    int blur_size_;
    int min_radius_;
    int min_brightness_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<VisionNode>());
    }
    catch (const std::exception& e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}