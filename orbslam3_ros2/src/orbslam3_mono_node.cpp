#include <chrono>
#include <memory>
#include <string>

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstring>
#include <cstdint>

#include "System.h"
#include "Tracking.h"

namespace {

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class OrbSlam3MonoNode final : public rclcpp::Node
{
public:
  explicit OrbSlam3MonoNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("orbslam3_mono", options)
  {
    vocabulary_file_ = this->declare_parameter<std::string>("vocabulary_file", "");
    settings_file_ = this->declare_parameter<std::string>("settings_file", "");
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    use_viewer_ = this->declare_parameter<bool>("use_viewer", true);

    // Performance knobs
    target_width_ = this->declare_parameter<int>("target_width", 640);
    target_height_ = this->declare_parameter<int>("target_height", 360);
    max_track_fps_ = this->declare_parameter<double>("max_track_fps", 10.0);

    // Publish map points at a lower rate to reduce load.
    map_publish_fps_ = this->declare_parameter<double>("map_publish_fps", 2.0);

    publish_pose_ = this->declare_parameter<bool>("publish_pose", false);
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera");

    if (vocabulary_file_.empty() || settings_file_.empty()) {
      throw std::runtime_error(
        "Parameters 'vocabulary_file' and 'settings_file' must be set.");
    }

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 vocabulary: %s", vocabulary_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 settings:   %s", settings_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing image:    %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Viewer:               %s", use_viewer_ ? "on" : "off");
    RCLCPP_INFO(this->get_logger(), "Target image:         %dx%d", target_width_, target_height_);
    RCLCPP_INFO(this->get_logger(), "Max track FPS:        %.2f", max_track_fps_);
    RCLCPP_INFO(this->get_logger(), "Map publish FPS:      %.2f", map_publish_fps_);

    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_file_, settings_file_, ORB_SLAM3::System::MONOCULAR, use_viewer_);

    if (publish_pose_) {
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
      path_msg_.header.frame_id = map_frame_;
    }

    // Publishers for visualization when Pangolin not available or headless
    frame_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/orbslam3/current_frame", 1);
    frame_overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/orbslam3/current_frame_overlay", 1);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orbslam3/map_points", 1);

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      std::bind(&OrbSlam3MonoNode::image_callback, this, std::placeholders::_1));
  }

  ~OrbSlam3MonoNode() override
  {
    // Stop ROS callbacks first, then shutdown SLAM threads.
    image_sub_.reset();
    pose_pub_.reset();
    path_pub_.reset();
    frame_pub_.reset();
    frame_overlay_pub_.reset();
    map_pub_.reset();

    if (slam_ && !slam_->isShutDown()) {
      slam_->Shutdown();
    }
    slam_.reset();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    // Drop frames if we're already processing too fast for this machine.
    if (max_track_fps_ > 0.0) {
      const auto now = std::chrono::steady_clock::now();
      const double min_dt = 1.0 / max_track_fps_;
      {
        std::lock_guard<std::mutex> lock(track_mutex_);
        if (last_track_wall_.time_since_epoch().count() != 0) {
          const std::chrono::duration<double> dt = now - last_track_wall_;
          if (dt.count() < min_dt) {
            return;
          }
        }
        last_track_wall_ = now;
      }
    }

    // Avoid cv_bridge: ROS2 Humble's cv_bridge is linked to OpenCV 4.5, while this
    // workspace's ORB-SLAM3 is linked to OpenCV 4.8. Mixing OpenCV versions can
    // segfault. We convert sensor_msgs/Image to cv::Mat manually.
    if (msg->data.empty() || msg->height == 0 || msg->width == 0 || msg->step == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty image received");
      return;
    }

    const size_t min_bytes = static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height);
    if (msg->data.size() < min_bytes) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Malformed image: data=%zu < step*height=%zu (w=%u h=%u step=%u enc=%s)",
        msg->data.size(), min_bytes, msg->width, msg->height, msg->step, msg->encoding.c_str());
      return;
    }

    cv::Mat im_gray;
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      const cv::Mat view(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
        const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
      im_gray = view.clone();
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      const cv::Mat view(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
      cv::cvtColor(view, im_gray, cv::COLOR_RGB2GRAY);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      const cv::Mat view(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()), static_cast<size_t>(msg->step));
      cv::cvtColor(view, im_gray, cv::COLOR_BGR2GRAY);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    // Optional resize for performance. Keep it simple: always resize to target if set.
    if (target_width_ > 0 && target_height_ > 0 &&
        (static_cast<int>(msg->width) != target_width_ || static_cast<int>(msg->height) != target_height_)) {
      cv::Mat resized;
      cv::resize(im_gray, resized, cv::Size(target_width_, target_height_), 0.0, 0.0, cv::INTER_AREA);
      im_gray = std::move(resized);
    }

    const double t = stamp_to_seconds(msg->header.stamp);

    Sophus::SE3f Tcw = slam_->TrackMonocular(im_gray, t);
    (void)Tcw;

    const int state = slam_->GetTrackingState();

    // Publish current frame (gray) for RViz/Image view
    if (frame_pub_ && frame_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::Image out_img;
      out_img.header = msg->header;
      out_img.header.frame_id = camera_frame_;
      out_img.height = static_cast<uint32_t>(im_gray.rows);
      out_img.width = static_cast<uint32_t>(im_gray.cols);
      out_img.encoding = "mono8";
      out_img.is_bigendian = 0;
      out_img.step = static_cast<uint32_t>(im_gray.cols);
      out_img.data.assign(im_gray.data, im_gray.data + (im_gray.rows * im_gray.cols));
      frame_pub_->publish(out_img);
    }

    // Publish an overlay that reflects ORB-SLAM3 processing: tracked keypoints + tracking state.
    if (frame_overlay_pub_ && frame_overlay_pub_->get_subscription_count() > 0) {
      cv::Mat overlay_bgr;
      cv::cvtColor(im_gray, overlay_bgr, cv::COLOR_GRAY2BGR);

      const std::vector<cv::KeyPoint> kps = slam_->GetTrackedKeyPointsUn();
      if (!kps.empty()) {
        cv::drawKeypoints(
          overlay_bgr, kps, overlay_bgr,
          cv::Scalar(0, 255, 0),
          cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
      }

      std::string state_text = "state=" + std::to_string(state);
      if (state == ORB_SLAM3::Tracking::OK || state == ORB_SLAM3::Tracking::OK_KLT) {
        state_text += " (OK)";
      }
      cv::putText(
        overlay_bgr, state_text,
        cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
        0.8, cv::Scalar(0, 255, 255), 2);

      sensor_msgs::msg::Image out_img;
      out_img.header = msg->header;
      out_img.header.frame_id = camera_frame_;
      out_img.height = static_cast<uint32_t>(overlay_bgr.rows);
      out_img.width = static_cast<uint32_t>(overlay_bgr.cols);
      out_img.encoding = sensor_msgs::image_encodings::BGR8;
      out_img.is_bigendian = 0;
      out_img.step = static_cast<uint32_t>(overlay_bgr.cols * 3);
      out_img.data.assign(
        overlay_bgr.data,
        overlay_bgr.data + (static_cast<size_t>(overlay_bgr.rows) * static_cast<size_t>(overlay_bgr.cols) * 3));
      frame_overlay_pub_->publish(out_img);
    }

    // Publish map points for RViz/PointCloud2 view (only when tracking is OK).
    // Publish map points for RViz/PointCloud2 view.
    // Do not hard-gate on tracking OK; RViz should keep showing the last known map even if
    // tracking is temporarily lost.
    {
      const bool want_map = map_pub_ && map_pub_->get_subscription_count() > 0;
      if (want_map && map_publish_fps_ > 0.0) {
        const auto now = std::chrono::steady_clock::now();
        const double min_dt = 1.0 / map_publish_fps_;
        std::lock_guard<std::mutex> lock(map_pub_mutex_);
        if (last_map_pub_wall_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_map_pub_wall_).count() >= min_dt)
        {
          last_map_pub_wall_ = now;
          publish_map_points(msg->header.stamp);
        }
      }
    }

    if (!publish_pose_) {
      (void)Tcw;
      return;
    }

    if (state != ORB_SLAM3::Tracking::OK && state != ORB_SLAM3::Tracking::OK_KLT) {
      return;
    }

    const Sophus::SE3f Twc = Tcw.inverse();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = map_frame_;

    pose.pose.position.x = Twc.translation().x();
    pose.pose.position.y = Twc.translation().y();
    pose.pose.position.z = Twc.translation().z();

    const Eigen::Quaternionf q = Twc.so3().unit_quaternion();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    pose_pub_->publish(pose);

    path_msg_.header.stamp = msg->header.stamp;
    path_msg_.poses.push_back(pose);
    path_pub_->publish(path_msg_);
  }

  // Helper: publish tracked map points as PointCloud2
  void publish_map_points(const builtin_interfaces::msg::Time & stamp)
  {
    if (!map_pub_ || !slam_) return;
    // Get tracked map points from ORB-SLAM3
    std::vector<ORB_SLAM3::MapPoint*> vMPs = slam_->GetTrackedMapPoints();
    const size_t max_points = 2000;
    std::vector<Eigen::Vector3f> points;
    points.reserve(std::min(vMPs.size(), max_points));

    for (ORB_SLAM3::MapPoint* pMP : vMPs) {
      if (!pMP) continue;
      if (pMP->isBad()) continue;
      points.push_back(pMP->GetWorldPos());
      if (points.size() >= max_points) break;
    }

    // If tracking is lost or map isn't initialized yet, ORB-SLAM3 may return empty.
    // Keep publishing the last non-empty cloud so RViz doesn't go blank.
    if (points.empty()) {
      if (last_nonempty_pc_.width > 0 && !last_nonempty_pc_.data.empty()) {
        sensor_msgs::msg::PointCloud2 pc = last_nonempty_pc_;
        pc.header.stamp = stamp;
        map_pub_->publish(pc);
      } else {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "No map points available yet (tracked points empty)");
      }
      return;
    }

    sensor_msgs::msg::PointCloud2 pc;
    pc.header.frame_id = map_frame_;
    pc.header.stamp = stamp;
    pc.is_bigendian = false;
    pc.is_dense = false;
    pc.height = 1;
    pc.width = static_cast<uint32_t>(points.size());

    pc.fields.resize(3);
    pc.fields[0].name = "x"; pc.fields[0].offset = 0; pc.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; pc.fields[0].count = 1;
    pc.fields[1].name = "y"; pc.fields[1].offset = 4; pc.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; pc.fields[1].count = 1;
    pc.fields[2].name = "z"; pc.fields[2].offset = 8; pc.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; pc.fields[2].count = 1;

    pc.point_step = 12;
    pc.row_step = pc.point_step * pc.width;
    pc.data.resize(pc.row_step);

    for (size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3f & pos = points[i];
      float x = pos.x();
      float y = pos.y();
      float z = pos.z();
      size_t base = i * pc.point_step;
      std::memcpy(&pc.data[base + 0], &x, sizeof(float));
      std::memcpy(&pc.data[base + 4], &y, sizeof(float));
      std::memcpy(&pc.data[base + 8], &z, sizeof(float));
    }

    last_nonempty_pc_ = pc;
    map_pub_->publish(pc);
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;

  std::string vocabulary_file_;
  std::string settings_file_;
  std::string image_topic_;
  bool use_viewer_{true};

  bool publish_pose_{false};
  std::string map_frame_;
  std::string camera_frame_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

  int target_width_{640};
  int target_height_{360};
  double max_track_fps_{10.0};
  std::mutex track_mutex_;
  std::chrono::steady_clock::time_point last_track_wall_{};

  double map_publish_fps_{2.0};
  std::mutex map_pub_mutex_;
  std::chrono::steady_clock::time_point last_map_pub_wall_{};

  sensor_msgs::msg::PointCloud2 last_nonempty_pc_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OrbSlam3MonoNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    // Use stderr here because logger may not be ready if constructor throws.
    fprintf(stderr, "orbslam3_mono_node fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
