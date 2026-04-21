#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <thread>

namespace
{
cv::Mat ensure_bgr8(const cv::Mat &frame)
{
    if (frame.empty()) {
        throw std::runtime_error("Cannot publish an empty frame");
    }

    if (frame.type() == CV_8UC3) {
        return frame;
    }

    cv::Mat converted;
    if (frame.type() == CV_8UC1) {
        cv::cvtColor(frame, converted, cv::COLOR_GRAY2BGR);
    } else if (frame.type() == CV_8UC4) {
        cv::cvtColor(frame, converted, cv::COLOR_BGRA2BGR);
    } else {
        throw std::runtime_error("Unsupported camera frame format");
    }

    return converted;
}

sensor_msgs::msg::Image mat_to_image_message(
    const cv::Mat &frame,
    const std_msgs::msg::Header &header)
{
    cv::Mat bgr = ensure_bgr8(frame);

    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = static_cast<sensor_msgs::msg::Image::_height_type>(bgr.rows);
    msg.width = static_cast<sensor_msgs::msg::Image::_width_type>(bgr.cols);
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.step);

    const auto *data_begin = bgr.data;
    const auto *data_end = data_begin + (bgr.step * bgr.rows);
    msg.data.assign(data_begin, data_end);

    return msg;
}
}  // namespace

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Declare parameter for the camera URL (RTSP, HTTP, or even local /dev/video0)
        this->declare_parameter<std::string>("video_source", "rtsp://192.168.1.100:554/stream");
        std::string video_source = this->get_parameter("video_source").as_string();

        auto qos = rclcpp::SensorDataQoS();
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos);

        RCLCPP_INFO(this->get_logger(), "Connecting to camera at: %s", video_source.c_str());

        // Open the video stream
        if (video_source.find("/dev/video") == 0) {
            // Для локальных USB-камер используем V4L2
            cap_.open(video_source, cv::CAP_V4L2);
            if (cap_.isOpened()) {
                // Жестко просим 1920x1080 и 30 FPS на уровне железа камеры
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
                cap_.set(cv::CAP_PROP_FPS, 30);
                // Запрашиваем формат MJPG, так как по USB 1080p@30fps обычно идет в нем
                cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            }
        } else {
            // Для RTSP и других сетевых потоков
            cap_.open(video_source, cv::CAP_FFMPEG);
        }
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source: %s", video_source.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully connected to the camera!");

        // Start a timer to read frames at approximately 30 FPS (~33ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_.read(frame) && !frame.empty()) {
            try {
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "camera_optical_frame";

                sensor_msgs::msg::Image img_msg = mat_to_image_message(frame, header);
                image_pub_->publish(img_msg);

                // Publish mock CameraInfo
                auto info_msg = sensor_msgs::msg::CameraInfo();
                info_msg.header = header;
                info_msg.height = img_msg.height;
                info_msg.width = img_msg.width;
                info_pub_->publish(info_msg);
            } catch (const std::exception &e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Failed to publish camera frame: %s",
                    e.what());
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to capture frame or stream ended.");
            // Optionally, try to reconnect here
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
