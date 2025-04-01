#ifndef ULTRASONIC_CONTROL_NODE_HPP
#define ULTRASONIC_CONTROL_NODE_HPP

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "mg400_msgs/srv/disable_robot.hpp"
#include "mg400_msgs/srv/enable_robot.hpp"
#include "mg400_msgs/srv/move_jog.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mg400_msgs/msg/robot_mode.hpp"

namespace ultrasonic_control
{
class UltrasonicControlNode : public rclcpp::Node {
public:
  explicit UltrasonicControlNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

private:
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_x_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_y_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_z_;
  rclcpp::Subscription<mg400_msgs::msg::RobotMode>::SharedPtr sub_robot_mode_;

  // Service Clients
  rclcpp::Client<mg400_msgs::srv::MoveJog>::SharedPtr move_jog_client_;
  rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_client_;
  rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mqtt_client_;

  // Sensor-Daten
  float distance_x_ = 50.0;
  float distance_y_ = 50.0;
  float distance_z_ = 50.0;

  // Roboter-Zustand
  std::shared_ptr<mg400_msgs::msg::RobotMode> current_robot_mode_;
  bool enable_call_in_progress_ = false;
  bool disable_call_in_progress_ = false;

  // Callbacks
  void distance_callback_x(const std_msgs::msg::Float32::SharedPtr msg);
  void distance_callback_y(const std_msgs::msg::Float32::SharedPtr msg);
  void distance_callback_z(const std_msgs::msg::Float32::SharedPtr msg);
  void robot_mode_callback(const mg400_msgs::msg::RobotMode::SharedPtr msg);

  // Steuerlogik
  void update_motion();
  void callMoveJog(const std::string & jog_mode);
  void enable_robot();
  void disable_robot();
};
}  // namespace ultrasonic_control

#endif  // ULTRASONIC_CONTROL_NODE_HPP
