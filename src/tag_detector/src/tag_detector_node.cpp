#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

        // Subscribe to camera image
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            std::bind(&TagDetectorNode::image_callback, this, std::placeholders::_1));

        // Publish tag pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tag_pose_camera", qos);
        
        // Initialize Aruco detector for AprilTag 36h11
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        detector_ = std::make_shared<cv::aruco::ArucoDetector>(dictionary, detectorParams);

        // Define the 3D coordinates of the marker corners
        // Assuming marker size is 0.15 meters (15 cm)
        float marker_size = 0.15f;
        float half_size = marker_size / 2.0f;
        obj_points_.push_back(cv::Point3f(-half_size,  half_size, 0));
        obj_points_.push_back(cv::Point3f( half_size,  half_size, 0));
        obj_points_.push_back(cv::Point3f( half_size, -half_size, 0));
        obj_points_.push_back(cv::Point3f(-half_size, -half_size, 0));

        RCLCPP_INFO(this->get_logger(), "Tag detector node started. Using OpenCV Aruco (AprilTag 36h11).");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV Mat
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            // The image from gphoto2 might be huge (e.g. 6192x4128).
            // Let's resize it to speed up detection (e.g., width = 1280)
            float scale = 1280.0f / frame.cols;
            cv::Mat resized_frame;
            cv::resize(frame, resized_frame, cv::Size(), scale, scale);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            
            // Detect markers
            detector_->detectMarkers(resized_frame, markerCorners, markerIds, rejectedCandidates);

            if (!markerIds.empty()) {
                RCLCPP_INFO(this->get_logger(), "Detected %zu tags! First tag ID: %d", markerIds.size(), markerIds[0]);

                // Approximate camera intrinsics for the RESIZED image
                // If you calibrate your camera, replace these with real values
                double fx = resized_frame.cols; // Guessing focal length
                double fy = resized_frame.cols;
                double cx = resized_frame.cols / 2.0;
                double cy = resized_frame.rows / 2.0;
                
                cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
                cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // Assume no distortion

                // Estimate pose for the first detected marker
                cv::Mat rvec, tvec;
                bool success = cv::solvePnP(obj_points_, markerCorners[0], cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

                if (success) {
                    // Convert rotation vector to quaternion
                    cv::Mat rotationMatrix;
                    cv::Rodrigues(rvec, rotationMatrix);
                    
                    // Convert rotation matrix to quaternion
                    double trace = rotationMatrix.at<double>(0,0) + rotationMatrix.at<double>(1,1) + rotationMatrix.at<double>(2,2);
                    double qw, qx, qy, qz;
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

                    auto pose_msg = geometry_msgs::msg::PoseStamped();
                    pose_msg.header.stamp = msg->header.stamp;
                    pose_msg.header.frame_id = "camera_link";
                    
                    pose_msg.pose.position.x = tvec.at<double>(0);
                    pose_msg.pose.position.y = tvec.at<double>(1);
                    pose_msg.pose.position.z = tvec.at<double>(2);
                    
                    pose_msg.pose.orientation.w = qw;
                    pose_msg.pose.orientation.x = qx;
                    pose_msg.pose.orientation.y = qy;
                    pose_msg.pose.orientation.z = qz;

                    pose_pub_->publish(pose_msg);
                    
                    RCLCPP_INFO(this->get_logger(), "Published Pose -> x: %.2f, y: %.2f, z: %.2f", 
                                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "No tags detected in this frame.");
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<cv::aruco::ArucoDetector> detector_;
    std::vector<cv::Point3f> obj_points_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
