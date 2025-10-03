#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <vector>
#include <string>

#include "dji_tello_interfaces/srv/tello_cmd.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class TelloAruco : public rclcpp::Node {
public:
  TelloAruco()
  : rclcpp::Node("dji_tello_aruco")
  {
    // QoS for image stream (best effort, low latency)
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    // Subscriber to Tello camera stream
    subscriber_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
      "tello/image_raw", qos,
      std::bind(&TelloAruco::tello_camera_callback, this, std::placeholders::_1));

    // Load both camera parameter files
    std::string home = std::getenv("HOME");

    // --- Front camera ---
    cv::FileStorage fs_front(
      home + std::string("/Desktop/dji_tello_ROS2/src/dji_tello_ros2/config/frontal_camera.yaml"),
      cv::FileStorage::READ);
    if (!fs_front.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open front camera YAML!");
    } else {
      fs_front["camera_matrix"] >> camera_matrix_front_;
      fs_front["distortion_coefficients"] >> dist_coeffs_front_;
      fs_front.release();
      RCLCPP_INFO(this->get_logger(), "Front camera parameters loaded.");
    }

    // --- Downward camera ---
    cv::FileStorage fs_down(
      home + std::string("/Desktop/dji_tello_ROS2/src/dji_tello_ros2/config/down_camera.yaml"),
      cv::FileStorage::READ);
    if (!fs_down.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open down camera YAML!");
    } else {
      fs_down["camera_matrix"] >> camera_matrix_down_;
      fs_down["distortion_coefficients"] >> dist_coeffs_down_;
      fs_down.release();
      RCLCPP_INFO(this->get_logger(), "Down camera parameters loaded.");
    }

    // Start with FRONT camera by default
    camera_matrix_ = camera_matrix_front_;
    dist_coeffs_   = dist_coeffs_front_;
    current_camera_ = "front_camera";

    // ROS2 service to enable/disable ArUco detection
    srv_ = this->create_service<dji_tello_interfaces::srv::TelloCmd>(
      "tello_aruco",
      std::bind(&TelloAruco::aruco_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            
    RCLCPP_INFO(this->get_logger(), "Tello ArUco node started (default: front camera)");
  }

private:
  // ROS objects
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_camera_;
  rclcpp::Service<dji_tello_interfaces::srv::TelloCmd>::SharedPtr srv_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // OpenCV matrices
  cv::Mat frame_;
  cv::Mat camera_matrix_, dist_coeffs_;      // active intrinsics
  cv::Mat camera_matrix_front_, dist_coeffs_front_;
  cv::Mat camera_matrix_down_, dist_coeffs_down_;

  // Flags
  bool enable_ = false;
  bool window_open_ = false;
  std::string current_camera_; // "front_camera" or "down_camera"

  // ArUco parameters
  cv::Ptr<cv::aruco::Dictionary> dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  cv::Ptr<cv::aruco::DetectorParameters> parameters_ =
      cv::aruco::DetectorParameters::create();
  std::vector<int> marker_ids_;
  std::vector<std::vector<cv::Point2f>> marker_corners_, rejected_candidates_;
  std::vector<cv::Vec3d> rvecs_, tvecs_;

  void tello_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!enable_) return;

    frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (frame_.empty()) return;

    // Detect ArUco markers
    cv::aruco::detectMarkers(frame_, dictionary_, marker_corners_,
                             marker_ids_, parameters_, rejected_candidates_);

    cv::Mat image_copy = frame_.clone();

    if (!marker_ids_.empty()) {
      cv::aruco::drawDetectedMarkers(image_copy, marker_corners_, marker_ids_);

      // Estimate pose with CURRENT camera intrinsics
      cv::aruco::estimatePoseSingleMarkers(marker_corners_, 0.10,
                                           camera_matrix_, dist_coeffs_,
                                           rvecs_, tvecs_);

      // Example: draw axis for marker ID 42
      auto it = std::find(marker_ids_.begin(), marker_ids_.end(), 42);
      if (it != marker_ids_.end()) {
        size_t pos = std::distance(marker_ids_.begin(), it);
        cv::drawFrameAxes(image_copy, camera_matrix_, dist_coeffs_,
                          rvecs_[pos], tvecs_[pos], 0.1);

        // Convert rvec to rotation matrix
        cv::Mat R;
        cv::Rodrigues(rvecs_[pos], R);

        cv::Mat tvec = (cv::Mat_<double>(3,1) 
            << tvecs_[pos][0], tvecs_[pos][1], tvecs_[pos][2]);

        cv::Mat R_inv = R.t();
        cv::Mat tvec_inv = -R_inv * tvec;

        // Convert to tf2 quaternion
        tf2::Matrix3x3 tf_rot(
          R_inv.at<double>(0,0), R_inv.at<double>(0,1), R_inv.at<double>(0,2),
          R_inv.at<double>(1,0), R_inv.at<double>(1,1), R_inv.at<double>(1,2),
          R_inv.at<double>(2,0), R_inv.at<double>(2,1), R_inv.at<double>(2,2));
        tf2::Quaternion q;
        tf_rot.getRotation(q);

        // Broadcast TF: marker -> camera
        geometry_msgs::msg::TransformStamped tf_camera;
        tf_camera.header.stamp = this->get_clock()->now();
        tf_camera.header.frame_id = "aruco";
        tf_camera.child_frame_id  = current_camera_; // dynamic: front_camera or down_camera
        tf_camera.transform.translation.x = tvec_inv.at<double>(0);
        tf_camera.transform.translation.y = tvec_inv.at<double>(1);
        tf_camera.transform.translation.z = tvec_inv.at<double>(2);
        tf_camera.transform.rotation.x = q.x();
        tf_camera.transform.rotation.y = q.y();
        tf_camera.transform.rotation.z = q.z();
        tf_camera.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_camera);
      }
    }

    // Show window
    if (cv::getWindowProperty("aruco", cv::WND_PROP_VISIBLE) < 0) {
      cv::namedWindow("aruco", cv::WINDOW_AUTOSIZE);
    }
    cv::imshow("aruco", image_copy);
    cv::waitKey(1);
    window_open_ = true;
  }

  void aruco_service_callback(
    const std::shared_ptr<dji_tello_interfaces::srv::TelloCmd::Request> req,
    std::shared_ptr<dji_tello_interfaces::srv::TelloCmd::Response> res)
  {
    if (req->command == "ARUCO") {
      enable_ = !enable_;
      res->success = true;

      if (!enable_ && window_open_) {
        cv::destroyAllWindows();
        window_open_ = false;
      }

      RCLCPP_INFO(this->get_logger(), "ARUCO detection %s",
                  enable_ ? "enabled" : "disabled");

    } else if (req->command == "SWITCHCAMERA") {
      // Toggle between FRONT and DOWN intrinsics
      if (current_camera_ == "front_camera") {
        camera_matrix_ = camera_matrix_down_;
        dist_coeffs_   = dist_coeffs_down_;
        current_camera_ = "down_camera";
      } else {
        camera_matrix_ = camera_matrix_front_;
        dist_coeffs_   = dist_coeffs_front_;
        current_camera_ = "front_camera";
      }
      res->success = true;
      RCLCPP_INFO(this->get_logger(), "Switched to %s", current_camera_.c_str());

    } else {
      res->success = false;
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s", req->command.c_str());
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelloAruco>());
  rclcpp::shutdown();
  return 0;
}
