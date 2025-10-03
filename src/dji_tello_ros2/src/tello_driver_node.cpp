#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>

#include "dji_tello_interfaces/srv/tello_cmd.hpp"
#include "dji_tello_Ros2/telloComunication.hpp"
#include "dji_tello_Ros2/telloInterface.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <memory>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class TelloDriverNode : public rclcpp::Node {
public:
  TelloDriverNode()
  : rclcpp::Node("dji_tello_driver")
  {
    // --- Connect to Tello drone ---
    if (ifc.begin(true) != 0 || ifc.connect() != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init Tello interface");
      return;
    }

    // --- QoS optimized for camera data ---
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    // --- Publishers ---
    publisher_battery_     = this->create_publisher<std_msgs::msg::Int32>("battery", 10);
    publisher_temperature_ = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);
    publisher_IMU_         = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    publisher_Tof_         = this->create_publisher<sensor_msgs::msg::Range>("tof", 10);
    publisher_Odom_        = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    publisher_Camera_      = this->create_publisher<sensor_msgs::msg::Image>("tello/image_raw", qos);
    publisher_info_camera  = this->create_publisher<sensor_msgs::msg::CameraInfo>("/tello/camera_info", 10);

    // --- Timers ---
    timer_tlmtry_ = this->create_wall_timer(100ms, std::bind(&TelloDriverNode::timer_callback_sensors_, this));   // 10 Hz telemetry
    timer_camera_ = this->create_wall_timer(200ms, std::bind(&TelloDriverNode::camera_callback, this));           // ~5 Hz camera
    timer_tf_     = this->create_wall_timer(100ms, std::bind(&TelloDriverNode::broadcast_odom_tf, this));         // 10 Hz TF

    // --- Subscribers ---
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_gamepad", 10,
      std::bind(&TelloDriverNode::tello_callback, this, std::placeholders::_1));

    // --- Camera Info Manager ---
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "tello_camera", yaml_url);
    cinfo_manager_down = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "tello_camera_down", down_url);
    // --- Flight service ---
    srv_ = this->create_service<dji_tello_interfaces::srv::TelloCmd>(
      "tello_command",
      std::bind(&TelloDriverNode::flightServiceFun, this,
                std::placeholders::_1, std::placeholders::_2));

    // --- TF Broadcasters ---
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Broadcast static transforms (front and down cameras relative to base_link)
    publish_static_transforms();

    RCLCPP_INFO(this->get_logger(), "Tello driver node started.");
  }

private:
  // --- Tello interface ---
  tello::interface ifc;
  cv::Mat frame;

  // --- Publishers ---
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_info_camera;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_battery_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_temperature_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_Tof_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_Odom_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_Camera_;

  // --- Subscribers and Services ---
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Service<dji_tello_interfaces::srv::TelloCmd>::SharedPtr srv_;

  // --- Camera Info ---
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_down;

  std::string yaml_url = "package://dji_tello_ros2/config/frontal_camera4ROS.yaml";
  std::string down_url = "package://dji_tello_ros2/config/down_camera4ROS.yaml";


  // --- Timers ---
  rclcpp::TimerBase::SharedPtr timer_tlmtry_;
  rclcpp::TimerBase::SharedPtr timer_camera_;
  rclcpp::TimerBase::SharedPtr timer_tf_;

  // --- TF Broadcasters ---
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // --- State variables ---
  bool enable_ = false;
  double x = 0.0, y = 0.0, z = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
  double last_roll_rad_ = 0.0, last_pitch_rad_ = 0.0, last_yaw_rad_ = 0.0;
  rclcpp::Time last_stamp_;
  bool has_last_angles_ = false;

  // =====================================================
  // === TELEOP CALLBACK ===
  // =====================================================
  void tello_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (enable_) {
      ifc.move(
        static_cast<int>(msg->linear.y * 100),  // left/right
        static_cast<int>(msg->linear.x * 100),  // forward/backward
        static_cast<int>(msg->linear.z * 100),  // up/down
        static_cast<int>(msg->angular.z * 60)   // yaw
      );
    }
  }

  // =====================================================
  // === FLIGHT SERVICE CALLBACK ===
  // =====================================================
  void flightServiceFun(
    const std::shared_ptr<dji_tello_interfaces::srv::TelloCmd::Request> req,
    std::shared_ptr<dji_tello_interfaces::srv::TelloCmd::Response> res)
  {
    const std::string &cmd = req->command;
    if (cmd == "TAKEOFF") {
      ifc.takeoff(); enable_ = true; res->success = true;
    } else if (cmd == "LAND") {
      ifc.land(); enable_ = false; res->success = true;
    } else if (cmd == "STREAMON") {
      ifc.video_on(); res->success = true;
    } else if (cmd == "STREAMOFF") {
      ifc.video_off(); res->success = true;
    } else if (cmd == "SWITCHCAMERA") {
      ifc.switchCamera(); res->success = true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s", cmd.c_str());
      res->success = false;
    }
  }

  // =====================================================
  // === TELEMETRY AND SENSORS ===
  // =====================================================
  void timer_callback_sensors_() {
    tello::Telemetry t = ifc.popTelemetry();
    if (t.stamp.time_since_epoch().count() == 0) return;

    rclcpp::Time now_time = to_ros_time(t.stamp);

    // --- IMU Orientation ---
    tf2::Quaternion q;
    q.setRPY(t.imu.roll * M_PI/180.0, t.imu.pitch * M_PI/180.0, t.imu.yaw * M_PI/180.0);

    // --- Odometry integration ---
    if (has_last_angles_) {
      double dt = (now_time - last_stamp_).seconds();
      if (dt > 1e-3) {
        double yaw_rad = t.imu.yaw * M_PI/180.0;
        double vx = t.imu.vgx / 100.0;  // m/s
        double vy = t.imu.vgy / 100.0;  // m/s
        // Transform body velocities into world frame
        double vx_world = vx * cos(yaw_rad) - vy * sin(yaw_rad);
        double vy_world = vx * sin(yaw_rad) + vy * cos(yaw_rad);
        x += vx_world * dt;
        y += vy_world * dt;
      }
    }
    last_stamp_ = now_time; has_last_angles_ = true;
    last_yaw_rad_ = t.imu.yaw * M_PI/180.0;

    // --- Publish Odometry ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = t.h * 0.01;
    z = odom.pose.pose.position.z ;

    odom.pose.pose.orientation.x = q.x();
    qx = q.x();
    odom.pose.pose.orientation.y = q.y();
    qy = q.y();
    odom.pose.pose.orientation.z = q.z();
    qz = q.z();
    odom.pose.pose.orientation.w = q.w();
    qw = q.w();
    publisher_Odom_->publish(odom);
  }

  // =====================================================
  // === CAMERA STREAM ===
  // =====================================================
  void camera_callback() {
    ifc.get_last_frame(frame);
    if (!frame.empty()) {
      cv::Mat resized_frame;
      cv::resize(frame, resized_frame, cv::Size(416, 416));
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_frame).toImageMsg();
      msg->header.stamp = this->now();
      msg->header.frame_id = "tello_camera";
      publisher_Camera_->publish(*msg);

      auto info_msg = cinfo_manager_->getCameraInfo();
      info_msg.header.stamp = msg->header.stamp;
      info_msg.header.frame_id = "tello_camera";
      publisher_info_camera->publish(info_msg);
    }
  }

  // =====================================================
  // === TF BROADCASTERS ===
  // =====================================================
  void publish_static_transforms() {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // base_link -> front_camera
    geometry_msgs::msg::TransformStamped tf_front;
    tf_front.header.stamp = this->get_clock()->now();
    tf_front.header.frame_id = "base_link";
    tf_front.child_frame_id  = "front_camera";
    tf_front.transform.translation.x = 0.03; // 3 cm forward
    tf_front.transform.translation.y = 0.0;
    tf_front.transform.translation.z = 0.0;
    tf2::Quaternion q1; q1.setRPY(0,0,0);
    tf_front.transform.rotation.x = q1.x();
    tf_front.transform.rotation.y = q1.y();
    tf_front.transform.rotation.z = q1.z();
    tf_front.transform.rotation.w = q1.w();
    transforms.push_back(tf_front);

    // base_link -> down_camera
    geometry_msgs::msg::TransformStamped tf_down;
    tf_down.header.stamp = this->get_clock()->now();
    tf_down.header.frame_id = "base_link";
    tf_down.child_frame_id  = "down_camera";
    tf_down.transform.translation.x = -0.012;
    tf_down.transform.translation.y = 0.0;
    tf_down.transform.translation.z = -0.0077; // 7 cm below
    tf2::Quaternion q2; q2.setRPY(0,0,0);
    tf_down.transform.rotation.x = q2.x();
    tf_down.transform.rotation.y = q2.y();
    tf_down.transform.rotation.z = q2.z();
    tf_down.transform.rotation.w = q2.w();
    transforms.push_back(tf_down);

    
    tf_static_broadcaster_->sendTransform(transforms);
  }

  void broadcast_odom_tf() {
    geometry_msgs::msg::TransformStamped tf_odom;
    tf_odom.header.stamp = this->get_clock()->now();
    tf_odom.header.frame_id = "odom_tello";
    tf_odom.child_frame_id  = "base_link";
    tf_odom.transform.translation.x = x;
    tf_odom.transform.translation.y = y;
    tf_odom.transform.translation.z = z;

    tf_odom.transform.rotation.x = qx;
    tf_odom.transform.rotation.y = qy;
    tf_odom.transform.rotation.z = qz;
    tf_odom.transform.rotation.w = qw;

    tf_broadcaster_->sendTransform(tf_odom);
  }

  // =====================================================
  // === TIME CONVERSION ===
  // =====================================================
  builtin_interfaces::msg::Time to_ros_time(const tello::steady_time & tp) { 
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
    builtin_interfaces::msg::Time t;
    t.sec  = static_cast<int32_t>(ns / 1000000000);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000);
    return t;
  }
};

// =====================================================
// === MAIN ===
// =====================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelloDriverNode>());
  rclcpp::shutdown();
  return 0;
}
