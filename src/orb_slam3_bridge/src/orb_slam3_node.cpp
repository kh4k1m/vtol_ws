// ROS 2 wrapper around ORB-SLAM3.
//
// Mirrors the contract the rest of the stack expects from any visual
// odometry source:
//
//   * subscribes to /camera/image_raw (+ optionally /ap/imu/raw for
//     IMU-monocular)
//   * publishes geometry_msgs/PoseStamped on the configured pose_topic
//     (default /vo/pose) - pose of the camera in ORB-SLAM3's local map
//   * publishes marker_interfaces/NavStatus on the configured
//     nav_status_topic (default /vo/nav_status) - source health,
//     covariance bounds and tracking state mapped onto our shared enum
//
// Like DPVO, ORB-SLAM3's local map is *not* gravity-aligned ENU - it is
// the SLAM frame established at the first frame. The vision fusion
// node aligns this local map to the tag map with a full SE(3) offset,
// so we just publish poses as ORB-SLAM3 produces them.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <marker_interfaces/msg/nav_status.hpp>

// ORB-SLAM3
#include "System.h"
#include "ImuTypes.h"
#include "Tracking.h"

namespace
{

cv::Mat image_msg_to_gray(const sensor_msgs::msg::Image & msg)
{
  if (msg.width == 0 || msg.height == 0 || msg.step == 0) {
    throw std::runtime_error("Received image with invalid geometry");
  }

  const auto required_bytes = static_cast<size_t>(msg.step) * msg.height;
  if (msg.data.size() < required_bytes) {
    throw std::runtime_error("Image data buffer is smaller than width/height/step");
  }

  cv::Mat gray;
  if (msg.encoding == sensor_msgs::image_encodings::MONO8) {
    cv::Mat wrapped(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC1,
      const_cast<unsigned char *>(msg.data.data()),
      msg.step);
    return wrapped.clone();
  }

  if (msg.encoding == sensor_msgs::image_encodings::BGR8) {
    cv::Mat wrapped(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC3,
      const_cast<unsigned char *>(msg.data.data()),
      msg.step);
    cv::cvtColor(wrapped, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }

  if (msg.encoding == sensor_msgs::image_encodings::RGB8) {
    cv::Mat wrapped(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC3,
      const_cast<unsigned char *>(msg.data.data()),
      msg.step);
    cv::cvtColor(wrapped, gray, cv::COLOR_RGB2GRAY);
    return gray;
  }

  if (msg.encoding == sensor_msgs::image_encodings::BGRA8) {
    cv::Mat wrapped(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC4,
      const_cast<unsigned char *>(msg.data.data()),
      msg.step);
    cv::cvtColor(wrapped, gray, cv::COLOR_BGRA2GRAY);
    return gray;
  }

  if (msg.encoding == sensor_msgs::image_encodings::RGBA8) {
    cv::Mat wrapped(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC4,
      const_cast<unsigned char *>(msg.data.data()),
      msg.step);
    cv::cvtColor(wrapped, gray, cv::COLOR_RGBA2GRAY);
    return gray;
  }

  throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class OrbSlam3Node : public rclcpp::Node
{
public:
  OrbSlam3Node()
  : Node("orb_slam3_node")
  {
    declare_parameter<std::string>("vocabulary_path", "");
    declare_parameter<std::string>("settings_path", "");
    declare_parameter<std::string>("sensor_type", "MONOCULAR");

    declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<std::string>("imu_topic", "/ap/imu/raw");
    declare_parameter<std::string>("pose_topic", "/vo/pose");
    declare_parameter<std::string>("nav_status_topic", "/vo/nav_status");
    declare_parameter<std::string>("path_topic", "/orb_slam3/path");
    declare_parameter<std::string>("frame_id", "orb_slam3_map");
    declare_parameter<bool>("enable_viewer", false);
    declare_parameter<bool>("publish_path", true);
    declare_parameter<int>("max_path_length", 2000);
    declare_parameter<int>("process_every_n", 1);
    declare_parameter<int>("log_every_n", 30);

    declare_parameter<double>("position_std_m", 0.20);
    declare_parameter<double>("rotation_std_rad", 0.10);
    declare_parameter<double>("velocity_std_m_s", 0.50);
    // Multiplier on std-dev when the tracker reports degraded state.
    declare_parameter<double>("degraded_std_multiplier", 2.5);

    vocabulary_path_ = get_parameter("vocabulary_path").as_string();
    settings_path_ = get_parameter("settings_path").as_string();
    const std::string sensor_str = get_parameter("sensor_type").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    pose_topic_ = get_parameter("pose_topic").as_string();
    nav_status_topic_ = get_parameter("nav_status_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    enable_viewer_ = get_parameter("enable_viewer").as_bool();
    publish_path_ = get_parameter("publish_path").as_bool();
    max_path_length_ = std::max(1, static_cast<int>(get_parameter("max_path_length").as_int()));
    process_every_n_ = std::max(1, static_cast<int>(get_parameter("process_every_n").as_int()));
    log_every_n_ = std::max(1, static_cast<int>(get_parameter("log_every_n").as_int()));

    pos_std_ = get_parameter("position_std_m").as_double();
    rot_std_ = get_parameter("rotation_std_rad").as_double();
    vel_std_ = get_parameter("velocity_std_m_s").as_double();
    degraded_mult_ = std::max(1.0, get_parameter("degraded_std_multiplier").as_double());

    if (vocabulary_path_.empty() || settings_path_.empty()) {
      throw std::runtime_error(
        "vocabulary_path and settings_path parameters are required");
    }

    sensor_type_ = parse_sensor_type(sensor_str);
    use_imu_ = (sensor_type_ == ORB_SLAM3::System::IMU_MONOCULAR);

    RCLCPP_INFO(
      get_logger(), "Loading ORB-SLAM3 vocabulary=%s settings=%s sensor=%s viewer=%s",
      vocabulary_path_.c_str(), settings_path_.c_str(),
      sensor_str.c_str(), enable_viewer_ ? "true" : "false");

    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_path_, settings_path_, sensor_type_, enable_viewer_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      std::bind(&OrbSlam3Node::on_image, this, std::placeholders::_1));

    if (use_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos,
        std::bind(&OrbSlam3Node::on_imu, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "IMU-MONO mode: subscribing to %s", imu_topic_.c_str());
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    nav_status_pub_ = create_publisher<marker_interfaces::msg::NavStatus>(
      nav_status_topic_, 10);
    if (publish_path_) {
      path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
      path_msg_.header.frame_id = frame_id_;
    }

    RCLCPP_INFO(
      get_logger(), "ORB-SLAM3 node ready. pose -> %s, nav_status -> %s",
      pose_topic_.c_str(), nav_status_topic_.c_str());
  }

  ~OrbSlam3Node() override
  {
    if (slam_) {
      try {
        slam_->Shutdown();
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "ORB-SLAM3 shutdown raised: %s", e.what());
      }
    }
  }

private:
  static ORB_SLAM3::System::eSensor parse_sensor_type(const std::string & s)
  {
    if (s == "MONOCULAR") return ORB_SLAM3::System::MONOCULAR;
    if (s == "IMU_MONOCULAR") return ORB_SLAM3::System::IMU_MONOCULAR;
    throw std::runtime_error("Unsupported sensor_type: " + s + " (use MONOCULAR or IMU_MONOCULAR)");
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.emplace_back(*msg);
    while (imu_buffer_.size() > 4096) {
      imu_buffer_.pop_front();
    }
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!slam_) {
      return;
    }

    image_count_++;
    if ((image_count_ - 1) % process_every_n_ != 0) {
      return;
    }

    cv::Mat gray;
    try {
      gray = image_msg_to_gray(*msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Failed to convert image: %s", e.what());
      return;
    }

    const double timestamp_sec = stamp_to_sec(msg->header.stamp);

    std::vector<ORB_SLAM3::IMU::Point> imu_meas;
    if (use_imu_) {
      imu_meas = drain_imu_until(timestamp_sec);
      if (last_image_timestamp_sec_ > 0.0 && imu_meas.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU buffer empty between image frames - is /ap/imu/raw publishing?");
      }
    }
    last_image_timestamp_sec_ = timestamp_sec;

    Sophus::SE3f Tcw;
    try {
      Tcw = slam_->TrackMonocular(gray, timestamp_sec, imu_meas);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ORB-SLAM3 TrackMonocular threw: %s", e.what());
      return;
    }

    const int tracking_state = slam_->GetTrackingState();
    publish_nav_status(msg->header.stamp, tracking_state);

    // ORB-SLAM3 returns T_world_camera = Tcw such that p_camera = Tcw * p_world.
    // For a /vo/pose convention we want the pose of the camera in the
    // world frame, i.e. T_wc = Tcw^-1.
    if (!is_pose_publishable(tracking_state)) {
      return;
    }

    const Sophus::SE3f Twc = Tcw.inverse();
    publish_pose(Twc, msg->header.stamp);
  }

  std::vector<ORB_SLAM3::IMU::Point> drain_imu_until(double image_timestamp_sec)
  {
    std::vector<ORB_SLAM3::IMU::Point> out;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    while (!imu_buffer_.empty()) {
      const auto & front = imu_buffer_.front();
      const double ts = stamp_to_sec(front.header.stamp);
      if (ts > image_timestamp_sec) {
        break;
      }
      out.emplace_back(
        static_cast<float>(front.linear_acceleration.x),
        static_cast<float>(front.linear_acceleration.y),
        static_cast<float>(front.linear_acceleration.z),
        static_cast<float>(front.angular_velocity.x),
        static_cast<float>(front.angular_velocity.y),
        static_cast<float>(front.angular_velocity.z),
        ts);
      imu_buffer_.pop_front();
    }
    return out;
  }

  static bool is_pose_publishable(int tracking_state)
  {
    using TS = ORB_SLAM3::Tracking;
    return tracking_state == TS::OK ||
           tracking_state == TS::OK_KLT ||
           tracking_state == TS::RECENTLY_LOST;
  }

  void publish_pose(const Sophus::SE3f & Twc, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();
    msg.pose.position.x = static_cast<double>(t.x());
    msg.pose.position.y = static_cast<double>(t.y());
    msg.pose.position.z = static_cast<double>(t.z());
    msg.pose.orientation.x = static_cast<double>(q.x());
    msg.pose.orientation.y = static_cast<double>(q.y());
    msg.pose.orientation.z = static_cast<double>(q.z());
    msg.pose.orientation.w = static_cast<double>(q.w());

    pose_pub_->publish(msg);

    if (publish_path_) {
      path_msg_.header.stamp = stamp;
      path_msg_.poses.push_back(msg);
      if (static_cast<int>(path_msg_.poses.size()) > max_path_length_) {
        path_msg_.poses.erase(
          path_msg_.poses.begin(),
          path_msg_.poses.begin() + (static_cast<int>(path_msg_.poses.size()) - max_path_length_));
      }
      path_pub_->publish(path_msg_);
    }

    pose_publish_count_++;
    if (pose_publish_count_ % log_every_n_ == 0) {
      RCLCPP_INFO(
        get_logger(),
        "ORB-SLAM3 pose #%lu: xyz=(%.3f, %.3f, %.3f) state=OK",
        pose_publish_count_, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
  }

  void publish_nav_status(const builtin_interfaces::msg::Time & stamp, int tracking_state)
  {
    using TS = ORB_SLAM3::Tracking;
    using marker_interfaces::msg::NavStatus;

    marker_interfaces::msg::NavStatus status;
    status.header.stamp = stamp;
    status.source = "orb_slam3";
    status.has_velocity = false;

    double quality_01 = 0.0;
    double std_scale = 1.0;
    switch (tracking_state) {
      case TS::OK:
        status.state = NavStatus::STATE_TRACKING;
        status.detail = "OK";
        quality_01 = 1.0;
        std_scale = 1.0;
        break;
      case TS::OK_KLT:
        status.state = NavStatus::STATE_TRACKING_DEGRADED;
        status.detail = "OK_KLT";
        quality_01 = 0.6;
        std_scale = degraded_mult_;
        break;
      case TS::RECENTLY_LOST:
        status.state = NavStatus::STATE_TRACKING_DEGRADED;
        status.detail = "RECENTLY_LOST";
        quality_01 = 0.3;
        std_scale = degraded_mult_;
        break;
      case TS::LOST:
        status.state = NavStatus::STATE_LOST;
        status.detail = "LOST";
        quality_01 = 0.0;
        std_scale = degraded_mult_ * 2.0;
        break;
      case TS::NOT_INITIALIZED:
        status.state = NavStatus::STATE_INITIALIZING;
        status.detail = "NOT_INITIALIZED";
        quality_01 = 0.0;
        break;
      case TS::NO_IMAGES_YET:
        status.state = NavStatus::STATE_NO_DATA;
        status.detail = "NO_IMAGES_YET";
        quality_01 = 0.0;
        break;
      default:
        status.state = NavStatus::STATE_UNKNOWN;
        status.detail = "STATE_" + std::to_string(tracking_state);
        quality_01 = 0.0;
        break;
    }

    status.quality_score = static_cast<float>(quality_01);
    status.position_std_m = static_cast<float>(pos_std_ * std_scale);
    status.rotation_std_rad = static_cast<float>(rot_std_ * std_scale);
    status.velocity_std_m_s = static_cast<float>(vel_std_ * std_scale);
    nav_status_pub_->publish(status);
  }

  // -- subscribers / publishers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<marker_interfaces::msg::NavStatus>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;

  // -- parameters
  std::string vocabulary_path_;
  std::string settings_path_;
  std::string image_topic_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string nav_status_topic_;
  std::string path_topic_;
  std::string frame_id_;
  bool enable_viewer_ = false;
  bool publish_path_ = true;
  int max_path_length_ = 2000;
  int process_every_n_ = 1;
  int log_every_n_ = 30;
  double pos_std_ = 0.2;
  double rot_std_ = 0.1;
  double vel_std_ = 0.5;
  double degraded_mult_ = 2.5;

  // -- state
  ORB_SLAM3::System::eSensor sensor_type_ = ORB_SLAM3::System::MONOCULAR;
  bool use_imu_ = false;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::mutex imu_mutex_;
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  double last_image_timestamp_sec_ = 0.0;
  unsigned long image_count_ = 0;
  unsigned long pose_publish_count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OrbSlam3Node>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "orb_slam3_node fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
