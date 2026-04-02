#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <marker_interfaces/msg/marker_detection_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

class TagDetectorNode : public rclcpp::Node
{
public:
    TagDetectorNode() : Node("tag_detector_node")
    {
        auto qos = rclcpp::SensorDataQoS();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            std::bind(&TagDetectorNode::image_callback, this, std::placeholders::_1));

        detections_pub_ = this->create_publisher<marker_interfaces::msg::MarkerDetectionArray>("/tag_detections", qos);
        
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        detector_ = std::make_shared<cv::aruco::ArucoDetector>(dictionary, detectorParams);

        RCLCPP_INFO(this->get_logger(), "Tag detector node started. Using original resolution and Sony A6700 intrinsics.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            
            detector_->detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

            if (!markerIds.empty()) {
                marker_interfaces::msg::MarkerDetectionArray msg_array;
                msg_array.header.stamp = msg->header.stamp;
                msg_array.header.frame_id = "camera_link";

                // ИСПОЛЬЗУЕМ КАЛИБРОВКУ SONY A6700
                // Оригинальное разрешение: 1920x1080
                // Параметры: [fx, fy, cx, cy] = [2941, 2934, 977, 567]
                // Дисторсия: [k1, k2, p1, p2, k3] = [-0.076268, 0.499635, 0.001955, 0.001634, -1.703305]
                
                double fx = 2941.0;
                double fy = 2934.0;
                double cx = 977.0;
                double cy = 567.0;

                cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
                cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.07626819355983554, 0.4996358213639364, 0.0019553139751148944, 0.0016349494242613198, -1.703305244612242);

                for (size_t i = 0; i < markerIds.size(); ++i) {
                    marker_interfaces::msg::MarkerDetection det_msg;
                    det_msg.id = markerIds[i];
                    
                    if (det_msg.id == 34 || det_msg.id == 6) det_msg.size_m = 0.04f;
                    else if (det_msg.id >= 1 && det_msg.id <= 4) det_msg.size_m = 0.16f;
                    else if (det_msg.id == 0) det_msg.size_m = 0.20f;
                    else if (det_msg.id >= 55 && det_msg.id <= 57) det_msg.size_m = 0.80f;
                    else continue;

                    float half_size = det_msg.size_m / 2.0f;
                    // ОРИГИНАЛЬНЫЙ ПОРЯДОК ТОЧЕК
                    std::vector<cv::Point3f> obj_points = {
                        cv::Point3f(-half_size,  half_size, 0),
                        cv::Point3f( half_size,  half_size, 0),
                        cv::Point3f( half_size, -half_size, 0),
                        cv::Point3f(-half_size, -half_size, 0)
                    };

                    cv::Mat rvec, tvec;
                    // ОРИГИНАЛЬНЫЙ SOLVEPNP
                    bool success = cv::solvePnP(obj_points, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

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
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<marker_interfaces::msg::MarkerDetectionArray>::SharedPtr detections_pub_;
    std::shared_ptr<cv::aruco::ArucoDetector> detector_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
