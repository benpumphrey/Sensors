#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ttt_msgs/msg/ball_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <algorithm>
#include <deque>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter("camera_id", "left");
        this->declare_parameter("min_area", 8);
        this->declare_parameter("max_area", 2000);
        this->declare_parameter("edge_margin", 10);
        this->declare_parameter("motion_threshold", 15);
        this->declare_parameter("min_contrast", 25);
        this->declare_parameter("dilate_iters", 1);
        this->declare_parameter("consistency_min", 2);
        this->declare_parameter("kf_gate_px", 150.0);
        this->declare_parameter("kf_reset_misses", 30);
        this->declare_parameter("kf_process_noise", 0.05);
        this->declare_parameter("hysteresis_r", 1);
        this->declare_parameter("hysteresis_thresh", 180);
        this->declare_parameter("table_roi", std::vector<int64_t>{});

        // NEW: Static mode for calibration
        this->declare_parameter("static_mode", false);

        camera_id_ = this->get_parameter("camera_id").as_string();
        min_area_ = this->get_parameter("min_area").as_int();
        max_area_ = this->get_parameter("max_area").as_int();
        edge_margin_ = this->get_parameter("edge_margin").as_int();
        motion_threshold_ = this->get_parameter("motion_threshold").as_int();
        min_contrast_ = this->get_parameter("min_contrast").as_int();
        dilate_iters_ = this->get_parameter("dilate_iters").as_int();
        consistency_min_ = this->get_parameter("consistency_min").as_int();
        kf_gate_px_sq_ = std::pow(this->get_parameter("kf_gate_px").as_double(), 2.0);
        kf_reset_misses_ = this->get_parameter("kf_reset_misses").as_int();
        kf_process_noise_ = (float)this->get_parameter("kf_process_noise").as_double();
        hysteresis_r_ = this->get_parameter("hysteresis_r").as_int();
        hysteresis_thresh_ = this->get_parameter("hysteresis_thresh").as_int();
        static_mode_ = this->get_parameter("static_mode").as_bool();

        auto roi_vec = this->get_parameter("table_roi").as_integer_array();
        if (roi_vec.size() == 8) {
            for (int i = 0; i < 4; i++)
                table_roi_.emplace_back((int)roi_vec[i * 2], (int)roi_vec[i * 2 + 1]);
            auto_detect_ = false;
        }
        else {
            auto_detect_ = true;
        }

        gaussian_filter_ = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);
        dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        open_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        detect_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));

        kf_ = cv::KalmanFilter(4, 2, 0, CV_32F);
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(kf_process_noise_));
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(0.1f));
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0f));

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult res;
                res.successful = true;
                for (const auto& p : params) {
                    if (p.get_name() == "static_mode") static_mode_ = p.as_bool();
                    if (p.get_name() == "min_contrast") min_contrast_ = p.as_int();
                    if (p.get_name() == "table_roi") {
                        auto vals = p.as_integer_array();
                        if (vals.empty()) {
                            table_roi_.clear();
                            roi_mask_ = cv::Mat();
                            auto_detect_ = true;
                            detect_attempt_ = 0;
                        }
                        else if (vals.size() >= 8 && vals.size() % 2 == 0) {
                            table_roi_.clear();
                            for (size_t i = 0; i < vals.size(); i += 2)
                                table_roi_.emplace_back((int)vals[i], (int)vals[i + 1]);
                            roi_mask_ = cv::Mat();
                            auto_detect_ = false;
                        }
                    }
                }
                return res;
            });

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/" + camera_id_ + "/image_raw", 10,
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<ttt_msgs::msg::BallDetection>("/ball_detection/" + camera_id_, 10);
    }

private:
    void autoDetectTable(const cv::Mat& gray) {
        cv::Mat dark;
        cv::threshold(gray, dark, 35, 255, cv::THRESH_BINARY_INV);
        cv::erode(dark, dark, detect_kernel_);
        cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, detect_kernel_);
        cv::morphologyEx(dark, dark, cv::MORPH_OPEN, detect_kernel_);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return;

        std::sort(contours.begin(), contours.end(), [](const auto& a, const auto& b) { return cv::contourArea(a) > cv::contourArea(b); });

        double frame_area = gray.cols * gray.rows;
        for (const auto& c : contours) {
            double ca = cv::contourArea(c);
            if (ca < frame_area * 0.10 || ca > frame_area * 0.92) continue;

            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, 0.03 * cv::arcLength(c, true), true);
            std::vector<cv::Point> hull;
            cv::convexHull(approx, hull);
            if (hull.size() < 4 || hull.size() > 6) continue;

            table_roi_ = hull;
            std::vector<int64_t> roi_param;
            for (const auto& pt : hull) { roi_param.push_back(pt.x); roi_param.push_back(pt.y); }
            this->set_parameter(rclcpp::Parameter("table_roi", roi_param));
            return;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat img;
        try { img = cv_bridge::toCvShare(msg, "mono8")->image; }
        catch (const cv_bridge::Exception& e) { return; }
        if (img.empty()) return;

        if (auto_detect_ && table_roi_.empty()) {
            if (++detect_attempt_ % 10 == 1) autoDetectTable(img);
        }

        if (!table_roi_.empty() && (roi_mask_.empty() || roi_mask_.size() != img.size())) {
            roi_mask_ = cv::Mat::zeros(img.size(), CV_8UC1);
            std::vector<cv::Point> extended_roi = table_roi_;
            for (const auto& pt : table_roi_) extended_roi.push_back(cv::Point(pt.x, 0));
            std::vector<cv::Point> extended_hull;
            cv::convexHull(extended_roi, extended_hull);
            cv::fillPoly(roi_mask_, std::vector<std::vector<cv::Point>>{extended_hull}, 255);
        }

        gpu_frame_.upload(img);
        gaussian_filter_->apply(gpu_frame_, gpu_blurred_);

        if (gpu_prev_blurred_.empty()) { gpu_blurred_.copyTo(gpu_prev_blurred_); return; }
        if (gpu_prev2_blurred_.empty()) { gpu_prev_blurred_.copyTo(gpu_prev2_blurred_); gpu_blurred_.copyTo(gpu_prev_blurred_); return; }

        if (static_mode_) {
            // Bypass subtraction completely - look at raw intensity
            cv::cuda::threshold(gpu_blurred_, gpu_motion_, min_contrast_, 255, cv::THRESH_BINARY);
            // FORCE Kalman to snap to the flashlight instantly
            kf_initialized_ = false;
        }
        else {
            cv::cuda::subtract(gpu_blurred_, gpu_prev2_blurred_, gpu_diff_);
            cv::cuda::threshold(gpu_diff_, gpu_motion_, motion_threshold_, 255, cv::THRESH_BINARY);
        }

        gpu_prev_blurred_.copyTo(gpu_prev2_blurred_);
        gpu_blurred_.copyTo(gpu_prev_blurred_);

        cv::Mat motion_mask;
        gpu_motion_.download(motion_mask);

        for (int i = 0; i < dilate_iters_; i++) cv::dilate(motion_mask, motion_mask, dilate_kernel_);
        cv::morphologyEx(motion_mask, motion_mask, cv::MORPH_OPEN, open_kernel_);

        if (!roi_mask_.empty()) cv::bitwise_and(motion_mask, roi_mask_, motion_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(motion_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        float best_x = -1.0f, best_y = -1.0f, best_r = 0.0f, best_contrast = 0.0f;
        bool found = false;

        for (const auto& contour : contours) {
            cv::Rect bbox = cv::boundingRect(contour);
            double area = bbox.width * bbox.height;
            if (area < min_area_ || area > max_area_) continue;

            cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
            if (center.x < edge_margin_ || center.x > img.cols - edge_margin_) continue;
            if (center.y < edge_margin_ || center.y > img.rows - edge_margin_) continue;

            cv::Mat roi = img(bbox);
            cv::Mat mask_roi = motion_mask(bbox);
            double brightness = cv::mean(roi, mask_roi)[0];
            if (brightness < min_contrast_) continue;

            if (brightness > best_contrast) {
                best_contrast = static_cast<float>(brightness);
                best_x = center.x;
                best_y = center.y;
                best_r = std::max(bbox.width, bbox.height) / 2.0f;
                found = true;
            }
        }

        // Hysteresis is skipped in static mode so it doesn't get confused
        if (!static_mode_ && !found && kf_initialized_) {
            int px = std::clamp((int)kf_.statePost.at<float>(0), 0, img.cols - 1);
            int py = std::clamp((int)kf_.statePost.at<float>(1), 0, img.rows - 1);
            cv::Rect box(
                std::max(0, px - hysteresis_r_), std::max(0, py - hysteresis_r_),
                std::min(2 * hysteresis_r_, img.cols - std::max(0, px - hysteresis_r_)),
                std::min(2 * hysteresis_r_, img.rows - std::max(0, py - hysteresis_r_)));
            double max_val; cv::Point max_loc;
            cv::minMaxLoc(img(box), nullptr, &max_val, nullptr, &max_loc);
            if (max_val >= hysteresis_thresh_) {
                best_x = (float)(box.x + max_loc.x); best_y = (float)(box.y + max_loc.y);
                best_r = 5.0f; best_contrast = (float)max_val; found = true;
            }
        }

        consistency_buf_.push_back(found);
        if ((int)consistency_buf_.size() > 3) consistency_buf_.pop_front();
        int hit_count = std::count(consistency_buf_.begin(), consistency_buf_.end(), true);
        bool consistent = (hit_count >= consistency_min_);

        double t_now = rclcpp::Time(msg->header.stamp).seconds();

        if (kf_initialized_) {
            float dt = std::clamp((float)(t_now - last_stamp_), 0.001f, 0.1f);
            last_stamp_ = t_now;
            kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1);
            kf_.predict();

            if (found) {
                float dx = best_x - kf_.statePre.at<float>(0), dy = best_y - kf_.statePre.at<float>(1);
                if (static_mode_ || (dx * dx + dy * dy <= kf_gate_px_sq_)) {
                    cv::Mat meas = (cv::Mat_<float>(2, 1) << best_x, best_y);
                    kf_.correct(meas);
                    miss_count_ = 0;
                }
                else {
                    kf_.statePre.copyTo(kf_.statePost);
                    miss_count_++;
                }
            }
            else {
                kf_.statePre.copyTo(kf_.statePost);
                miss_count_++;
            }

            if (miss_count_ >= kf_reset_misses_) {
                kf_initialized_ = false; miss_count_ = 0; consistency_buf_.clear();
            }
        }
        else if (consistent && found) {
            kf_.statePost = (cv::Mat_<float>(4, 1) << best_x, best_y, 0.0f, 0.0f);
            cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(100.0f));
            kf_initialized_ = true; last_stamp_ = t_now; miss_count_ = 0;
        }

        auto det_msg = ttt_msgs::msg::BallDetection();
        det_msg.header = msg->header;
        det_msg.x = -1.0f; det_msg.y = -1.0f;

        if (kf_initialized_) {
            det_msg.x = kf_.statePost.at<float>(0); det_msg.y = kf_.statePost.at<float>(1);
            det_msg.radius = found ? best_r : 0.0f; det_msg.confidence = found ? best_contrast : 0.0f;
        }
        detection_pub_->publish(det_msg);
    }

    std::string camera_id_;
    int min_area_, max_area_, edge_margin_, motion_threshold_, min_contrast_, dilate_iters_;
    bool auto_detect_ = false, static_mode_ = false;
    int detect_attempt_ = 0;
    std::vector<cv::Point> table_roi_;
    cv::Mat roi_mask_, dilate_kernel_, open_kernel_, detect_kernel_;

    cv::cuda::GpuMat gpu_frame_, gpu_blurred_, gpu_prev_blurred_, gpu_prev2_blurred_, gpu_diff_, gpu_motion_;
    cv::Ptr<cv::cuda::Filter> gaussian_filter_;

    int consistency_min_; std::deque<bool> consistency_buf_;
    cv::KalmanFilter kf_;
    bool kf_initialized_ = false; int miss_count_ = 0; double last_stamp_ = 0.0;
    float kf_gate_px_sq_, kf_process_noise_;
    int kf_reset_misses_, hysteresis_r_, hysteresis_thresh_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<ttt_msgs::msg::BallDetection>::SharedPtr detection_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}