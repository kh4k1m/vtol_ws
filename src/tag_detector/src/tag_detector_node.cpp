#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <marker_interfaces/msg/marker_detection_array.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>
#include <map>
#include <stdexcept>

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

class TagDetectorNode : public rclcpp::Node
{
public:
    TagDetectorNode() : Node("tag_detector_node")
    {
        // Declare parameters
        this->declare_parameter<std::vector<double>>("camera_matrix_data", {2941.0, 0.0, 977.0, 0.0, 2934.0, 567.0, 0.0, 0.0, 1.0});
        this->declare_parameter<std::vector<double>>("distortion_coefficients_data", {-0.076268, 0.499635, 0.001955, 0.001634, -1.703305});
        this->declare_parameter<double>("default_marker_size", 0.8);
        this->declare_parameter<std::vector<int64_t>>("marker_ids", {3, 4, 5, 10, 11});
        this->declare_parameter<std::vector<double>>("marker_sizes", {0.8, 0.8, 0.8, 1.5, 0.2});

        // Read parameters
        auto cam_data = this->get_parameter("camera_matrix_data").as_double_array();
        auto dist_data = this->get_parameter("distortion_coefficients_data").as_double_array();
        default_marker_size_ = this->get_parameter("default_marker_size").as_double();
        auto m_ids = this->get_parameter("marker_ids").as_integer_array();
        auto m_sizes = this->get_parameter("marker_sizes").as_double_array();

        RCLCPP_INFO(this->get_logger(), "Loaded camera matrix with %zu elements", cam_data.size());

        // Build camera matrix and distortion safely
        cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
        if (cam_data.size() >= 9) {
            for (int i = 0; i < 9; ++i) cameraMatrix_.at<double>(i / 3, i % 3) = cam_data[i];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera_matrix_data size! Using identity matrix.");
        }
        
        distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        for (size_t i = 0; i < std::min((size_t)5, dist_data.size()); ++i) distCoeffs_.at<double>(i) = dist_data[i];

        // Build marker sizes map
        for (size_t i = 0; i < std::min(m_ids.size(), m_sizes.size()); ++i) {
            marker_sizes_[m_ids[i]] = m_sizes[i];
        }

        auto qos = rclcpp::SensorDataQoS();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            std::bind(&TagDetectorNode::image_callback, this, std::placeholders::_1));

        detections_pub_ = this->create_publisher<marker_interfaces::msg::MarkerDetectionArray>("/tag_detections", qos);
        
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        detector_ = std::make_shared<cv::aruco::ArucoDetector>(dictionary, detectorParams);

        RCLCPP_INFO(this->get_logger(), "Tag detector node started with dynamic parameters.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            if (msg->width == 0 || msg->height == 0 || msg->step == 0) {
                RCLCPP_WARN(this->get_logger(), "Received invalid image: %ux%u, step %u", msg->width, msg->height, msg->step);
                return;
            }

            cv::Mat frame = image_msg_to_bgr(*msg);
            
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image frame!");
                return;
            }
            
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            
            try {
                detector_->detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "detectMarkers exception: %s", e.what());
                return;
            }

            if (!markerIds.empty()) {
                marker_interfaces::msg::MarkerDetectionArray msg_array;
                msg_array.header.stamp = msg->header.stamp;
                msg_array.header.frame_id = msg->header.frame_id.empty() ? "camera_optical_frame" : msg->header.frame_id;

                for (size_t i = 0; i < markerIds.size(); ++i) {
                    marker_interfaces::msg::MarkerDetection det_msg;
                    det_msg.id = markerIds[i];
                    
                    // Get size from map or use default
                    if (marker_sizes_.count(det_msg.id)) {
                        det_msg.size_m = marker_sizes_[det_msg.id];
                    } else {
                        det_msg.size_m = default_marker_size_;
                    }

                    float half_size = det_msg.size_m / 2.0f;
                    std::vector<cv::Point3f> obj_points = {
                        cv::Point3f(-half_size,  half_size, 0),
                        cv::Point3f( half_size,  half_size, 0),
                        cv::Point3f( half_size, -half_size, 0),
                        cv::Point3f(-half_size, -half_size, 0)
                    };

                    cv::Mat rvec, tvec;
                    bool success = false;
                    try {
                        // ИСПРАВЛЕНИЕ: Гарантируем, что cameraMatrix_ и distCoeffs_ имеют правильный тип и размер
                        cv::Mat camMat, dist;
                        cameraMatrix_.convertTo(camMat, CV_64F);
                        distCoeffs_.convertTo(dist, CV_64F);
                        
                        success = cv::solvePnP(obj_points, markerCorners[i], camMat, dist, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
                    } catch (const cv::Exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "solvePnP exception: %s", e.what());
                        continue;
                    }

                    if (success) {
                        cv::Mat rotationMatrix;
                        cv::Rodrigues(rvec, rotationMatrix);
                        
                        double trace = rotationMatrix.at<double>(0,0) + rotationMatrix.at<double>(1,1) + rotationMatrix.at<double>(2,2);
                        double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
                        
                        if (trace > 0) {
                            double s = 0.5 / sqrt(trace + 1.0);
                            qw = 0.25 / s;
                            qx = (rotationMatrix.at<double>(2,1) - rotationMatrix.at<double>(1,2)) * s;
                            qy = (rotationMatrix.at<double>(0,2) - rotationMatrix.at<double>(2,0)) * s;
                            qz = (rotationMatrix.at<double>(1,0) - rotationMatrix.at<double>(0,1)) * s;
                        } else {
                            if (rotationMatrix.at<double>(0,0) > rotationMatrix.at<double>(1,1) && rotationMatrix.at<double>(0,0) > rotationMatrix.at<double>(2,2)) {
                                double s = 2.0 * sqrt(1.0 + rotationMatrix.at<double>(0,0) - rotationMatrix.at<double>(1,1) - rotationMatrix.at<double>(2,2));
                                qw = (rotationMatrix.at<double>(2,1) - rotationMatrix.at<double>(1,2)) / s;
                                qx = 0.25 * s;
                                qy = (rotationMatrix.at<double>(0,1) + rotationMatrix.at<double>(1,0)) / s;
                                qz = (rotationMatrix.at<double>(0,2) + rotationMatrix.at<double>(2,0)) / s;
                            } else if (rotationMatrix.at<double>(1,1) > rotationMatrix.at<double>(2,2)) {
                                double s = 2.0 * sqrt(1.0 + rotationMatrix.at<double>(1,1) - rotationMatrix.at<double>(0,0) - rotationMatrix.at<double>(2,2));
                                qw = (rotationMatrix.at<double>(0,2) - rotationMatrix.at<double>(2,0)) / s;
                                qx = (rotationMatrix.at<double>(0,1) + rotationMatrix.at<double>(1,0)) / s;
                                qy = 0.25 * s;
                                qz = (rotationMatrix.at<double>(1,2) + rotationMatrix.at<double>(2,1)) / s;
                            } else {
                                double s = 2.0 * sqrt(1.0 + rotationMatrix.at<double>(2,2) - rotationMatrix.at<double>(0,0) - rotationMatrix.at<double>(1,1));
                                qw = (rotationMatrix.at<double>(1,0) - rotationMatrix.at<double>(0,1)) / s;
                                qx = (rotationMatrix.at<double>(0,2) + rotationMatrix.at<double>(2,0)) / s;
                                qy = (rotationMatrix.at<double>(1,2) + rotationMatrix.at<double>(2,1)) / s;
                                qz = 0.25 * s;
                            }
                        }

                        det_msg.pose_camera_marker.position.x = tvec.at<double>(0);
                        det_msg.pose_camera_marker.position.y = tvec.at<double>(1);
                        det_msg.pose_camera_marker.position.z = tvec.at<double>(2);
                        det_msg.pose_camera_marker.orientation.w = qw;
                        det_msg.pose_camera_marker.orientation.x = qx;
                        det_msg.pose_camera_marker.orientation.y = qy;
                        det_msg.pose_camera_marker.orientation.z = qz;
                        
                        det_msg.decision_margin = 1.0f;
                        det_msg.pose_valid = true;
                        
                        msg_array.detections.push_back(det_msg);
                    }
                }

                if (!msg_array.detections.empty()) {
                    detections_pub_->publish(msg_array);
                }
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception in callback: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion exception in callback: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<marker_interfaces::msg::MarkerDetectionArray>::SharedPtr detections_pub_;
    std::shared_ptr<cv::aruco::ArucoDetector> detector_;

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    double default_marker_size_;
    std::map<int, double> marker_sizes_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
