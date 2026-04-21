#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{
cv::Mat image_msg_to_bgr(const sensor_msgs::msg::Image &msg)
{
    if (msg.width == 0 || msg.height == 0 || msg.step == 0) {
        throw std::runtime_error("Received image with invalid geometry");
    }

    const auto required_bytes = static_cast<size_t>(msg.step) * msg.height;
    if (msg.data.size() < required_bytes) {
        throw std::runtime_error("Image data buffer is smaller than width/height/step");
    }

    if (msg.encoding == sensor_msgs::image_encodings::BGR8) {
        cv::Mat wrapped(
            static_cast<int>(msg.height),
            static_cast<int>(msg.width),
            CV_8UC3,
            const_cast<unsigned char *>(msg.data.data()),
            msg.step);
        return wrapped.clone();
    }

    cv::Mat converted;
    if (msg.encoding == sensor_msgs::image_encodings::RGB8) {
        cv::Mat wrapped(
            static_cast<int>(msg.height),
            static_cast<int>(msg.width),
            CV_8UC3,
            const_cast<unsigned char *>(msg.data.data()),
            msg.step);
        cv::cvtColor(wrapped, converted, cv::COLOR_RGB2BGR);
        return converted;
    }

    if (msg.encoding == sensor_msgs::image_encodings::MONO8) {
        cv::Mat wrapped(
            static_cast<int>(msg.height),
            static_cast<int>(msg.width),
            CV_8UC1,
            const_cast<unsigned char *>(msg.data.data()),
            msg.step);
        cv::cvtColor(wrapped, converted, cv::COLOR_GRAY2BGR);
        return converted;
    }

    if (msg.encoding == sensor_msgs::image_encodings::BGRA8) {
        cv::Mat wrapped(
            static_cast<int>(msg.height),
            static_cast<int>(msg.width),
            CV_8UC4,
            const_cast<unsigned char *>(msg.data.data()),
            msg.step);
        cv::cvtColor(wrapped, converted, cv::COLOR_BGRA2BGR);
        return converted;
    }

    if (msg.encoding == sensor_msgs::image_encodings::RGBA8) {
        cv::Mat wrapped(
            static_cast<int>(msg.height),
            static_cast<int>(msg.width),
            CV_8UC4,
            const_cast<unsigned char *>(msg.data.data()),
            msg.step);
        cv::cvtColor(wrapped, converted, cv::COLOR_RGBA2BGR);
        return converted;
    }

    throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
}
}  // namespace

class VideoRecorderNode : public rclcpp::Node
{
public:
    VideoRecorderNode() : Node("video_recorder_node")
    {
        this->declare_parameter<std::string>("output_path", "/home/nvidia/vtol_ws/flight_video.mp4");
        this->declare_parameter<double>("fps", 30.0);
        
        output_path_ = this->get_parameter("output_path").as_string();
        fps_ = this->get_parameter("fps").as_double();
        
        // Create CSV file for timestamps
        std::string csv_path = output_path_.substr(0, output_path_.find_last_of('.')) + "_timestamps.csv";
        csv_file_.open(csv_path);
        csv_file_ << "frame_index,ros_time_sec,ros_time_nanosec" << std::endl;
        
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            std::bind(&VideoRecorderNode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Video recorder initialized. Saving to %s", output_path_.c_str());
    }
    
    ~VideoRecorderNode()
    {
        if (video_writer_.isOpened()) {
            video_writer_.release();
        }
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = image_msg_to_bgr(*msg);
            
            if (!video_writer_.isOpened()) {
                int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
                cv::Size frame_size(frame.cols, frame.rows);
                video_writer_.open(output_path_, fourcc, fps_, frame_size, true);
                
                if (!video_writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open video writer!");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Started recording video at %dx%d", frame.cols, frame.rows);
            }
            
            video_writer_.write(frame);
            
            // Log timestamp
            csv_file_ << frame_count_ << "," 
                      << msg->header.stamp.sec << "," 
                      << msg->header.stamp.nanosec << std::endl;
            
            frame_count_++;
            
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
    std::string output_path_;
    double fps_;
    int frame_count_ = 0;
    std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
