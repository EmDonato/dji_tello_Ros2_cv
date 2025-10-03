#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "dji_tello_interfaces/srv/tello_cmd.hpp"

#include <memory>
#include <string>
#include <chrono>
using namespace std::chrono_literals;

class GamepadNode : public rclcpp::Node {
public:
  GamepadNode()
  : rclcpp::Node("gamepad_tello_node"), enable_(false), stream_(false), prev_button_x_(0)
  {
    // Publisher for velocity commands coming from gamepad
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_gamepad", 10);

    // Subscriber for joystick messages
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&GamepadNode::joy_callback, this, std::placeholders::_1));

    // Client for Tello command service
    client_ = this->create_client<dji_tello_interfaces::srv::TelloCmd>("tello_command");
    client_yolo = this->create_client<dji_tello_interfaces::srv::TelloCmd>("tello_yolo");
    client_aruco = this->create_client<dji_tello_interfaces::srv::TelloCmd>("tello_aruco");

  }

private:
  bool enable_;       // true if drone is airborne (after takeoff)
  bool stream_;       // true if video stream is ON
  int prev_button_x_; // to detect edge (press/release) for X button

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Client<dji_tello_interfaces::srv::TelloCmd>::SharedPtr client_;
  rclcpp::Client<dji_tello_interfaces::srv::TelloCmd>::SharedPtr client_yolo;
  rclcpp::Client<dji_tello_interfaces::srv::TelloCmd>::SharedPtr client_aruco;


  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto twist_msg = geometry_msgs::msg::Twist();

    // === Button mapping ===
    if (msg->buttons[0] == 1) {  
      // A button → Takeoff
      call_service("TAKEOFF");
      enable_ = true;
    } 
    else if (msg->buttons[1] == 1) {  
      // B button → Land
      call_service("LAND");
      enable_ = false;
    } 
    else if (msg->buttons[2] == 1 && prev_button_x_ == 0) {  
      // X button → toggle video stream ON/OFF
      if (stream_) {
        call_service("STREAMOFF");
        RCLCPP_INFO(this->get_logger(), "Stream OFF requested");
      } else {
        call_service("STREAMON");
        RCLCPP_INFO(this->get_logger(), "Stream ON requested");
      }
      stream_ = !stream_;
    }
    prev_button_x_ = msg->buttons[2]; // update X button state

    if (msg->buttons[3] == 1) {  
      // Y button → Switch camera
      call_service("SWITCHCAMERA");
      call_service_aruco("SWITCHCAMERA");
    }

    if (msg->axes[7] == 1) {     
      // D-Pad UP → Show camera
      call_service("SHOWCAMERA");
    }
    if (msg->axes[7] == -1) {     
      // D-Pad UP → Show camera
      call_service_yolo("YOLO");
    }
    if (msg->axes[6] == 1) {     
      // D-Pad UP → Show camera
      call_service_aruco("ARUCO");
    }

    // === Movement control (only if airborne) ===
    if (enable_) {
      twist_msg.linear.x  = msg->axes[1];                        // forward / backward
      twist_msg.linear.y  = msg->axes[2];                        // left / right
      twist_msg.linear.z  = (msg->axes[5]) / 2 - (msg->axes[4]) / 2; // up / down (triggers)
      twist_msg.angular.z = -msg->buttons[4] + msg->buttons[5];  // yaw (LB / RB)
      publisher_->publish(twist_msg);
    }
  }

  void call_service(const std::string &command) {
    auto request = std::make_shared<dji_tello_interfaces::srv::TelloCmd::Request>();
    request->command = command;

    if (!client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available", client_->get_service_name());
      return;
    }

    // Async call with response callback
    auto future = client_->async_send_request(
      request,
      [this, command](rclcpp::Client<dji_tello_interfaces::srv::TelloCmd>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "[OK] Command %s executed", command.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "[FAIL] Command %s failed", command.c_str());
        }
      }
    );
    (void)future; // avoid "unused variable" warning
  }

    void call_service_yolo(const std::string &command) {
    auto request = std::make_shared<dji_tello_interfaces::srv::TelloCmd::Request>();
    request->command = command;

    if (!client_yolo->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available", client_yolo->get_service_name());
      return;
    }

    // Async call with response callback
    auto future = client_yolo->async_send_request(
      request,
      [this, command](rclcpp::Client<dji_tello_interfaces::srv::TelloCmd>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "[OK] Command %s executed", command.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "[FAIL] Command %s failed", command.c_str());
        }
      }
    );
    (void)future; // avoid "unused variable" warning
  }

  void call_service_aruco(const std::string &command) {
    auto request = std::make_shared<dji_tello_interfaces::srv::TelloCmd::Request>();
    request->command = command;

    if (!client_aruco->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available", client_aruco->get_service_name());
      return;
    }

    // Async call with response callback
    auto future = client_aruco->async_send_request(
      request,
      [this, command](rclcpp::Client<dji_tello_interfaces::srv::TelloCmd>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "[OK] Command %s executed", command.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "[FAIL] Command %s failed", command.c_str());
        }
      }
    );
    (void)future; // avoid "unused variable" warning
  }



};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadNode>());
  rclcpp::shutdown();
  return 0;
}
