#include "ultrasonic_control/ultrasonic_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "mg400_msgs/srv/disable_robot.hpp"
#include "mg400_msgs/srv/enable_robot.hpp"
#include "mg400_msgs/srv/move_jog.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mg400_msgs/msg/robot_mode.hpp"

namespace ultrasonic_control
{

UltrasonicControlNode::UltrasonicControlNode(const rclcpp::NodeOptions & node_options)
: Node("ultrasonic_control_node", node_options)
{
    move_jog_client_ = this->create_client<mg400_msgs::srv::MoveJog>("/mg400/move_jog");
    enable_robot_client_ = this->create_client<mg400_msgs::srv::EnableRobot>("/mg400/enable_robot");
    disable_robot_client_ = this->create_client<mg400_msgs::srv::DisableRobot>("/mg400/disable_robot");

    sub_x_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/distance/x", 10,
        std::bind(&UltrasonicControlNode::distance_callback_x, this, std::placeholders::_1));

    sub_y_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/distance/y", 10,
        std::bind(&UltrasonicControlNode::distance_callback_y, this, std::placeholders::_1));

    sub_z_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sensor/distance/z", 10,
        std::bind(&UltrasonicControlNode::distance_callback_z, this, std::placeholders::_1));

    sub_robot_mode_ = this->create_subscription<mg400_msgs::msg::RobotMode>(
        "/mg400/robot_mode", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&UltrasonicControlNode::robot_mode_callback, this, std::placeholders::_1));

    current_robot_mode_ = std::make_shared<mg400_msgs::msg::RobotMode>();
}

void UltrasonicControlNode::distance_callback_x(const std_msgs::msg::Float32::SharedPtr msg) {
    distance_x_ = msg->data;
    update_motion();
}

void UltrasonicControlNode::distance_callback_y(const std_msgs::msg::Float32::SharedPtr msg) {
    distance_y_ = msg->data;
    update_motion();
}

void UltrasonicControlNode::distance_callback_z(const std_msgs::msg::Float32::SharedPtr msg) {
    distance_z_ = msg->data;
    update_motion();
}

void UltrasonicControlNode::robot_mode_callback(const mg400_msgs::msg::RobotMode::SharedPtr msg) {
    current_robot_mode_ = msg;
}

void UltrasonicControlNode::update_motion() {
    using RobotMode = mg400_msgs::msg::RobotMode;

    const auto & mode = current_robot_mode_->robot_mode;
    if (mode != RobotMode::ENABLE && mode != RobotMode::DISABLED) {
        return;
    }

    if (distance_x_ < 10.0 || distance_y_ < 10.0 || distance_z_ < 10.0) {
        if (mode != RobotMode::ENABLE && !enable_call_in_progress_) {
            enable_robot();
        }
    } else if (distance_x_ > 20.0 && distance_y_ > 20.0 && distance_z_ > 20.0) {
        if (mode != RobotMode::DISABLED && !disable_call_in_progress_) {
            disable_robot();
        }
        return;
    }

    if (mode != RobotMode::ENABLE && mode != RobotMode::JOG && mode != RobotMode::RUNNING) {
        RCLCPP_WARN(this->get_logger(), "Current state is not jog acceptable");
        return;
    }

    std::string jog_mode = "";
    if (distance_x_ < 10.0) {
        jog_mode = "X_NEGATIVE";
    } else if (distance_x_ > 10.0) {
        jog_mode = "X_POSITIVE";
    } else if (distance_y_ < 10.0) {
        jog_mode = "Y_NEGATIVE";
    } else if (distance_y_ > 10.0) {
        jog_mode = "Y_POSITIVE";
    } else if (distance_z_ < 20.0) {
        jog_mode = "Z_NEGATIVE";
    } else if (distance_z_ > 10.0) {
        jog_mode = "Z_POSITIVE";
    }

    callMoveJog(jog_mode);
}

void UltrasonicControlNode::callMoveJog(const std::string & jog_mode) {
    if (!move_jog_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "MoveJog service not available");
        return;
    }

    auto req = std::make_shared<mg400_msgs::srv::MoveJog::Request>();
    req->jog.jog_mode = jog_mode;

    auto future = move_jog_client_->async_send_request(req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
}

void UltrasonicControlNode::enable_robot() {
    if (!enable_robot_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "EnableRobot service not available");
        return;
    }

    auto req = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();
    enable_call_in_progress_ = true;
    auto future = enable_robot_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        const auto & result = future.get();
        if (result->result) {
            RCLCPP_INFO(this->get_logger(), "Aktivierung erfolgreich");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Aktivierung fehlgeschlagen, error_id: %d", result->error_id);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Aktivierungsservice fehlgeschlagen");
    }
    enable_call_in_progress_ = false;
}

void UltrasonicControlNode::disable_robot() {
    if (!disable_robot_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "DisableRobot service not available");
        return;
    }

    auto req = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();
    disable_call_in_progress_ = true;
    auto future = disable_robot_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        const auto & result = future.get();
        if (result->result) {
            RCLCPP_INFO(this->get_logger(), "Deaktivierung erfolgreich");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Deaktivierung fehlgeschlagen, error_id: %d", result->error_id);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Deaktivierungsservice fehlgeschlagen");
    }
    disable_call_in_progress_ = false;
}

}  // namespace ultrasonic_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ultrasonic_control::UltrasonicControlNode>());
    rclcpp::shutdown();
    return 0;
}
