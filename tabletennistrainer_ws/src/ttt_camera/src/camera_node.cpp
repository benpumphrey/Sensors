#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class CameraNode : public rclcpp::Node
{
public:		
    CameraNode(): Node("camera_node")
    {
        // parameters for node
        this->declare_parameter("device", "/dev/video0");
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("width", 1280); 
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 120);
        this->declare_parameter("show_window", true);

        // getters
        std::string device = this->get_parameter("device").as_string();
        camera_id_ = this->get_parameter("camera_id").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();
        show_window_ = this->get_parameter("show_window").as_bool();  

        RCLCPP_INFO(this->get_logger(), "Opening camera: %s", device.c_str());

        // Open camera using V4L2
        cap_.open(device, cv::CAP_V4L2);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", device.c_str());
            throw std::runtime_error("Camera open failed");
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera opened successfully!");

		// set camera properties
		cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
		cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
		cap_.set(cv::CAP_PROP_FPS, fps);
		cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

		// get actual vals
		int actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
		int actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
		double actual_fps = cap_.get(cv::CAP_PROP_FPS);
        
        RCLCPP_INFO(this->get_logger(), "Camera opened successfully:");
        RCLCPP_INFO(this->get_logger(), "  Device: %s", device.c_str());
        RCLCPP_INFO(this->get_logger(), "  Camera ID: %s", camera_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d (requested: %dx%d)", 
                    actual_width, actual_height, width, height);
        RCLCPP_INFO(this->get_logger(), "  FPS: %.1f (requested: %d)", actual_fps, fps);
        RCLCPP_INFO(this->get_logger(), "  Format: GRAY8 (Monochrome)");
        
        // Create window if requested
        if (show_window_) {
            window_name_ = "Camera: " + camera_id_;
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            RCLCPP_INFO(this->get_logger(), "Display window enabled: %s", window_name_.c_str());
        }
        
        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", 10);
        
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/" + camera_id_ + "/camera_info", 10);
        
        // Timer to capture frames
        auto period = std::chrono::microseconds(static_cast<int>(1000000.0 / fps));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&CameraNode::captureFrame, this));
        
        // FPS counter timer
        fps_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CameraNode::reportFPS, this));
        
        RCLCPP_INFO(this->get_logger(), "Publishing to:");
        RCLCPP_INFO(this->get_logger(), "  /camera/%s/image_raw", camera_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  /camera/%s/camera_info", camera_id_.c_str());
    }
    
    ~CameraNode()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
        if (show_window_) {
            cv::destroyWindow(window_name_);
        }
    }

private:
    void captureFrame()
    {
        cv::Mat frame;
        
        // Try to read frame
        bool success = cap_.read(frame);
        
        if (!success || frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Failed to read frame");
            return;
        }
        
		cv::flip(frame, frame, 0); 

        // Validate frame
        if (frame.rows <= 0 || frame.cols <= 0 || frame.data == nullptr) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Invalid frame: %dx%d", frame.cols, frame.rows);
            return;
        }
        
        try {
            // Show window FIRST
            if (show_window_) {
                cv::imshow(window_name_, frame);
                cv::waitKey(1);
            }
            
            // Create ROS Image message MANUALLY
            auto msg = std::make_shared<sensor_msgs::msg::Image>();
            msg->header.stamp = this->now();
            msg->header.frame_id = camera_id_ + "_optical_frame";
            msg->height = frame.rows;
            msg->width = frame.cols;
            msg->encoding = "mono8";
            msg->is_bigendian = false;
            msg->step = frame.cols;
            
            // Copy image data
            size_t size = msg->step * msg->height;
            msg->data.resize(size);
            
            if (frame.isContinuous()) {
                memcpy(&msg->data[0], frame.data, size);
            } else {
                for (int i = 0; i < frame.rows; ++i) {
                    memcpy(&msg->data[i * msg->step], frame.ptr(i), msg->step);
                }
            }
            
            // Publish image
            image_pub_->publish(*msg);
            
            // Publish camera info
            auto info_msg = sensor_msgs::msg::CameraInfo();
            info_msg.header = msg->header;
            info_msg.height = frame.rows;
            info_msg.width = frame.cols;
            camera_info_pub_->publish(info_msg);
            
            frame_count_++;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in captureFrame: %s", e.what());
        }
    }
    
    void reportFPS()
    {
        double fps = frame_count_;
        frame_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Publishing at %.1f FPS", fps);
    }
    
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr fps_timer_;
    
    std::string camera_id_;
    std::string window_name_;
    bool show_window_;
    size_t frame_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<CameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_node"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}