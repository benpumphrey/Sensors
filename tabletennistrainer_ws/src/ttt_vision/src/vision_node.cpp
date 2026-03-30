#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>           // Fix 1: was CompressedImage
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <algorithm>
#include <deque>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("min_circularity", 0.65);
        this->declare_parameter("min_radius", 4);
        this->declare_parameter("max_radius", 25);
        this->declare_parameter("edge_margin", 25);
        this->declare_parameter("motion_threshold", 30);
        this->declare_parameter("min_brightness",   200);
        this->declare_parameter("dilate_iters",       2);
        this->declare_parameter("table_roi", std::vector<int64_t>{});
        this->declare_parameter("consistency_min",    2);   // Fix 3: min hits in 3-frame window
        this->declare_parameter("kf_gate_px",        60.0); // Fix 4: innovation gate radius (px)
        this->declare_parameter("kf_reset_misses",   10);   // Fix 4: consecutive misses before KF reset

        camera_id_        = this->get_parameter("camera_id").as_string();
        min_circularity_  = this->get_parameter("min_circularity").as_double();
        min_radius_       = this->get_parameter("min_radius").as_int();
        max_radius_       = this->get_parameter("max_radius").as_int();
        edge_margin_      = this->get_parameter("edge_margin").as_int();
        motion_threshold_ = this->get_parameter("motion_threshold").as_int();
        min_brightness_   = this->get_parameter("min_brightness").as_int();
        dilate_iters_     = this->get_parameter("dilate_iters").as_int();
        consistency_min_  = this->get_parameter("consistency_min").as_int();
        kf_gate_px_sq_    = std::pow(this->get_parameter("kf_gate_px").as_double(), 2.0);
        kf_reset_misses_  = this->get_parameter("kf_reset_misses").as_int();

        auto roi_vec = this->get_parameter("table_roi").as_integer_array();
        if (roi_vec.size() == 8) {
            for (int i = 0; i < 4; i++)
                table_roi_.emplace_back((int)roi_vec[i*2], (int)roi_vec[i*2+1]);
            auto_detect_ = false;
            RCLCPP_INFO(this->get_logger(),
                "[%s] Manual table ROI: (%ld,%ld) (%ld,%ld) (%ld,%ld) (%ld,%ld)",
                camera_id_.c_str(),
                roi_vec[0],roi_vec[1], roi_vec[2],roi_vec[3],
                roi_vec[4],roi_vec[5], roi_vec[6],roi_vec[7]);
        } else {
            auto_detect_ = true;
            RCLCPP_INFO(this->get_logger(),
                "[%s] Auto table-detect ON", camera_id_.c_str());
        }

        gaussian_filter_ = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);
        dilate_kernel_   = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        open_kernel_     = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        detect_kernel_   = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));

        // Fix 4: Kalman filter — state [x, y, vx, vy], measurement [x, y]
        kf_ = cv::KalmanFilter(4, 2, 0, CV_32F);
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0);
        cv::setIdentity(kf_.processNoiseCov,     cv::Scalar::all(1e-4f));
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(0.1f));
        cv::setIdentity(kf_.errorCovPost,        cv::Scalar::all(1.0f));

        // Fix 1: subscribe to uncompressed image_raw — no JPEG artifacts
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", 10,
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>(
            "/ball_detection/" + camera_id_, 10);

        RCLCPP_INFO(this->get_logger(),
            "[%s] Vision node online | motion>%d AND brightness>%d | "
            "consistency_min=%d kf_gate=%.0fpx kf_reset=%d",
            camera_id_.c_str(), motion_threshold_, min_brightness_,
            consistency_min_, std::sqrt(kf_gate_px_sq_), kf_reset_misses_);
    }

private:
    // ── Auto-detect table from dark surface ──────────────────────────────────
    void autoDetectTable(const cv::Mat& gray) {
        cv::Mat dark;
        cv::threshold(gray, dark, 60, 255, cv::THRESH_BINARY_INV);
        cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, detect_kernel_);
        cv::morphologyEx(dark, dark, cv::MORPH_OPEN,  detect_kernel_);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return;

        std::sort(contours.begin(), contours.end(),
            [](const auto& a, const auto& b){ return cv::contourArea(a) > cv::contourArea(b); });

        double min_area = gray.cols * gray.rows * 0.12;
        for (const auto& c : contours) {
            if (cv::contourArea(c) < min_area) break;
            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, 0.03 * cv::arcLength(c, true), true);
            std::vector<cv::Point> hull;
            cv::convexHull(approx, hull);
            if (hull.size() < 4 || hull.size() > 6) continue;
            table_roi_ = hull;
            RCLCPP_INFO(this->get_logger(),
                "[%s] Table auto-detected: %zu corners, %.0f%% of frame — ROI locked",
                camera_id_.c_str(), hull.size(),
                100.0 * cv::contourArea(hull) / (gray.cols * gray.rows));
            return;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[%s] Auto-detect: no table yet (attempt %d)", camera_id_.c_str(), detect_attempt_);
    }

    // ── Main callback ─────────────────────────────────────────────────────────
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Fix 1: decode from raw image via cv_bridge — no JPEG compression artifacts
        cv::Mat img;
        try {
            img = cv_bridge::toCvShare(msg, "mono8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        if (img.empty()) return;

        if (auto_detect_ && table_roi_.empty()) {
            if (++detect_attempt_ % 10 == 1)
                autoDetectTable(img);
        }

        if (!table_roi_.empty() && (roi_mask_.empty() ||
            roi_mask_.size() != img.size())) {
            roi_mask_ = cv::Mat::zeros(img.size(), CV_8UC1);
            cv::fillPoly(roi_mask_,
                std::vector<std::vector<cv::Point>>{table_roi_}, 255);
        }

        // ── GPU pipeline ──────────────────────────────────────────────────────
        gpu_frame_.upload(img);
        gaussian_filter_->apply(gpu_frame_, gpu_blurred_);
        cv::cuda::threshold(gpu_frame_, gpu_bright_, min_brightness_, 255, cv::THRESH_BINARY);

        // Fix 2: 3-frame diff — seed the two-frame buffer before processing
        if (gpu_prev_blurred_.empty()) {
            gpu_blurred_.copyTo(gpu_prev_blurred_);
            return;
        }
        if (gpu_prev_prev_blurred_.empty()) {
            gpu_prev_blurred_.copyTo(gpu_prev_prev_blurred_);
            gpu_blurred_.copyTo(gpu_prev_blurred_);
            return;
        }

        // Diff against t-2: 2x stronger motion signal than single-frame diff
        cv::cuda::absdiff(gpu_blurred_, gpu_prev_prev_blurred_, gpu_diff_);
        cv::cuda::threshold(gpu_diff_, gpu_motion_, motion_threshold_, 255, cv::THRESH_BINARY);

        // Shift buffer: t-1 → t-2, current → t-1
        gpu_prev_blurred_.copyTo(gpu_prev_prev_blurred_);
        gpu_blurred_.copyTo(gpu_prev_blurred_);

        cv::cuda::bitwise_and(gpu_motion_, gpu_bright_, gpu_combined_);

        cv::Mat mask;
        gpu_combined_.download(mask);

        // ── CPU morphology ────────────────────────────────────────────────────
        for (int i = 0; i < dilate_iters_; i++)
            cv::dilate(mask, mask, dilate_kernel_);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel_);

        if (!roi_mask_.empty())
            cv::bitwise_and(mask, roi_mask_, mask);

        // ── Raw detection (best candidate blob) ───────────────────────────────
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        float raw_x = -1.0f, raw_y = -1.0f, raw_radius = 0.0f, raw_conf = 0.0f;
        bool raw_valid = false;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 5) continue;
            double perimeter = cv::arcLength(contour, true);
            if (perimeter < 5) continue;
            double circularity = (4.0 * M_PI * area) / (perimeter * perimeter);
            if (circularity <= min_circularity_) continue;

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < min_radius_ || radius > max_radius_) continue;
            if (center.x < edge_margin_ || center.x > img.cols - edge_margin_) continue;
            if (center.y < edge_margin_ || center.y > img.rows - edge_margin_) continue;

            if (circularity > raw_conf) {
                raw_conf   = static_cast<float>(circularity);
                raw_x      = center.x;
                raw_y      = center.y;
                raw_radius = radius;
                raw_valid  = true;
            }
        }

        // Fix 3: Temporal consistency gate — require ball visible in 2 of last 3 frames
        consistency_buf_.push_back(raw_valid);
        if ((int)consistency_buf_.size() > 3) consistency_buf_.pop_front();
        int hit_count = std::count(consistency_buf_.begin(), consistency_buf_.end(), true);
        bool consistent = (hit_count >= consistency_min_);

        // Fix 4: Kalman filter predict/update
        double t_now = rclcpp::Time(msg->header.stamp).seconds();

        if (kf_initialized_) {
            float dt = std::clamp(static_cast<float>(t_now - last_stamp_), 0.001f, 0.1f);
            last_stamp_ = t_now;

            kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, dt,  0,
                0, 1,  0, dt,
                0, 0,  1,  0,
                0, 0,  0,  1);

            kf_.predict();  // advances statePre; statePost updated on correct()

            if (raw_valid) {
                // Innovation gate: reject detections too far from prediction
                float dx = raw_x - kf_.statePre.at<float>(0);
                float dy = raw_y - kf_.statePre.at<float>(1);
                if (dx*dx + dy*dy <= kf_gate_px_sq_) {
                    cv::Mat meas = (cv::Mat_<float>(2, 1) << raw_x, raw_y);
                    kf_.correct(meas);
                    miss_count_ = 0;
                } else {
                    // Gated out — copy prediction to statePost so next predict() is stable
                    kf_.statePre.copyTo(kf_.statePost);
                    miss_count_++;
                }
            } else {
                kf_.statePre.copyTo(kf_.statePost);
                miss_count_++;
            }

            if (miss_count_ >= kf_reset_misses_) {
                kf_initialized_ = false;
                miss_count_ = 0;
                consistency_buf_.clear();
            }
        } else if (consistent && raw_valid) {
            // First consistent detection — seed the filter
            kf_.statePost = (cv::Mat_<float>(4, 1) << raw_x, raw_y, 0.0f, 0.0f);
            cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(100.0f));
            kf_initialized_ = true;
            last_stamp_ = t_now;
            miss_count_ = 0;
        }

        // ── Publish ───────────────────────────────────────────────────────────
        auto det_msg = ttt_msgs::msg::BallDetection();
        det_msg.header = msg->header;
        det_msg.x = -1.0f; det_msg.y = -1.0f;

        if (kf_initialized_) {
            det_msg.x          = kf_.statePost.at<float>(0);
            det_msg.y          = kf_.statePost.at<float>(1);
            det_msg.radius     = raw_valid ? raw_radius : 0.0f;
            det_msg.confidence = raw_valid ? raw_conf   : 0.0f;
        }

        detection_pub_->publish(det_msg);
    }

    std::string camera_id_;
    int min_radius_, max_radius_, edge_margin_, motion_threshold_, min_brightness_, dilate_iters_;
    double min_circularity_;
    bool auto_detect_ = false;
    int detect_attempt_ = 0;
    std::vector<cv::Point> table_roi_;
    cv::Mat roi_mask_, dilate_kernel_, open_kernel_, detect_kernel_;

    // Fix 2: added gpu_prev_prev_blurred_ for 3-frame diff
    cv::cuda::GpuMat gpu_frame_, gpu_blurred_, gpu_prev_blurred_, gpu_prev_prev_blurred_;
    cv::cuda::GpuMat gpu_diff_, gpu_motion_, gpu_bright_, gpu_combined_;
    cv::Ptr<cv::cuda::Filter> gaussian_filter_;

    // Fix 3: temporal consistency
    int consistency_min_;
    std::deque<bool> consistency_buf_;

    // Fix 4: Kalman filter
    cv::KalmanFilter kf_;
    bool kf_initialized_ = false;
    int miss_count_ = 0;
    double last_stamp_ = 0.0;
    float kf_gate_px_sq_;
    int kf_reset_misses_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;  // Fix 1
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
