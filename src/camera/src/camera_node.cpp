#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>

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

        RCLCPP_INFO(this->get_logger(), "Connecting to Ethernet camera at: %s", video_source.c_str());

        // Open the video stream
        cap_.open(video_source, cv::CAP_FFMPEG);
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
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_link";
            
            // Convert OpenCV Mat to ROS Image message
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            image_pub_->publish(*img_msg);
            
            // Publish mock CameraInfo
            auto info_msg = sensor_msgs::msg::CameraInfo();
            info_msg.header = header;
            info_msg.height = frame.rows;
            info_msg.width = frame.cols;
            info_pub_->publish(info_msg);
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
