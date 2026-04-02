#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <chrono>

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
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
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
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
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
