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

        RCLCPP_INFO(this->get_logger(), "Vision Node [%s] initialized at full resolution.", camera_id_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "c_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        cv::Mat annotated_frame;
        cv::cvtColor(frame, annotated_frame, cv::COLOR_GRAY2BGR);

        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(blur_size_, blur_size_), 0, 0);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
                         frame.rows / 8, 100, 20,
                         min_radius_, 0);

        auto detection_msg = ttt_msgs::msg::BallDetection();
        detection_msg.header = msg->header;

        int brightest_idx = -1;
        int max_brightness = 0;

        if (!circles.empty()) {
            for (size_t i = 0; i < circles.size(); i++) {
                cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);

                int x = std::max(0, center.x - radius);
                int y = std::max(0, center.y - radius);
                int w = std::min(2 * radius, frame.cols - x);
                int h = std::min(2 * radius, frame.rows - y);

                if (w > 0 && h > 0) {
                    cv::Rect roi(x, y, w, h);
                    cv::Scalar avg = cv::mean(frame(roi));
                    if (avg[0] > max_brightness) {
                        max_brightness = avg[0];
                        brightest_idx = i;
                    }
                }
            }
        }

        if (brightest_idx != -1 && max_brightness >= min_brightness_) {
            float best_x = circles[brightest_idx][0];
            float best_y = circles[brightest_idx][1];
            float best_r = circles[brightest_idx][2];

            detection_msg.x = best_x;
            detection_msg.y = best_y;
            detection_msg.radius = best_r;
            detection_msg.confidence = max_brightness / 255.0;

            if (debug_mode_) {
                cv::Point center(cvRound(best_x), cvRound(best_y));
                int r = cvRound(best_r);
                cv::circle(annotated_frame, center, r, cv::Scalar(0, 255, 0), 2);

                std::string hud = "BRIGHT: " + std::to_string(max_brightness);
                cv::putText(annotated_frame, hud, cv::Point(center.x - r, center.y - r - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        } else {
            detection_msg.x = -1.0;
            detection_msg.y = -1.0;
            detection_msg.radius = 0.0;
            detection_msg.confidence = 0.0;
        }

        detection_pub_->publish(detection_msg);

        if (debug_mode_) {
            if (cv::getWindowProperty("Vision Tracker", cv::WND_PROP_VISIBLE) < 1) {
                cv::namedWindow("Vision Tracker", cv::WINDOW_NORMAL);
            }
            cv::imshow("Vision Tracker", annotated_frame);
            cv::waitKey(1);
        }
    }

    // --- Private Members ---
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
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in VisionNode: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
