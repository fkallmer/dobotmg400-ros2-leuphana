#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>  // For Trigger service
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

const std::string ADDRESS = "tcp://localhost:1883";
const std::string CLIENT_ID = "mqtt_bridge";
const std::vector<std::string> MQTT_TOPICS = {"sensor/distance/x", "sensor/distance/y", "sensor/distance/z"};

class MqttRos2Bridge : public rclcpp::Node, public mqtt::callback
{
public:
    MqttRos2Bridge()
    : Node("mqtt_bridge"),
      client_(ADDRESS, CLIENT_ID)
    {
        // ------------------
        // ROS 2 Publishers
        // ------------------
        pub_x_ = this->create_publisher<std_msgs::msg::Float32>("/sensor/distance/x", 10);
        pub_y_ = this->create_publisher<std_msgs::msg::Float32>("/sensor/distance/y", 10);
        pub_z_ = this->create_publisher<std_msgs::msg::Float32>("/sensor/distance/z", 10);

        // --------------------------------------------------
        // ROS 2 Service: /mqtt_bridge/mqtt_node
        // --------------------------------------------------
        // This allows other nodes to check if the bridge is "started" or "available."
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/mqtt_bridge/mqtt_node",
            std::bind(&MqttRos2Bridge::handle_mqtt_bridge_service, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // ------------------
        // MQTT Setup
        // ------------------
        client_.set_callback(*this);
        mqtt::connect_options conn_opts;
        conn_opts.set_keep_alive_interval(10);
        conn_opts.set_clean_session(true);

        try {
            client_.connect(conn_opts)->wait();
            RCLCPP_INFO(this->get_logger(), "Successfully connected to MQTT broker at %s", ADDRESS.c_str());
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
        }

        for (const auto &topic : MQTT_TOPICS) {
            client_.subscribe(topic, 0);
            RCLCPP_INFO(this->get_logger(), "Subscribed to MQTT topic: %s", topic.c_str());
        }
    }

    // --------------------------------------------------
    // MQTT callback for arriving messages
    // --------------------------------------------------
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        try {
            std_msgs::msg::Float32 ros_msg;
            std::string payload = msg->to_string();

            if (payload.empty()) {
                return;
            }

            // Parse JSON message
            json j = json::parse(payload);

            // Check the topic and JSON field(s)
            if (msg->get_topic() == "sensor/distance/x" && j.contains("x")) {
                ros_msg.data = j["x"].get<float>();
                pub_x_->publish(ros_msg);

            } else if (msg->get_topic() == "sensor/distance/y" && j.contains("y")) {
                ros_msg.data = j["y"].get<float>();
                pub_y_->publish(ros_msg);

            } else if (msg->get_topic() == "sensor/distance/z" && j.contains("z")) {
                ros_msg.data = j["z"].get<float>();
                pub_z_->publish(ros_msg);

            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Received MQTT message on %s, but key was missing",
                            msg->get_topic().c_str());
            }

        } catch (const json::parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing MQTT message: %s", e.what());
        }
    }

private:
    // --------------------------------------------------
    // Service callback handler
    // --------------------------------------------------
    void handle_mqtt_bridge_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // We might do more checks here, e.g. is MQTT connected?
        // For now, we simply respond "true."
        RCLCPP_INFO(this->get_logger(), "Service /mqtt_bridge/mqtt_node was called");
        response->success = true;
        response->message = "MQTT Bridge is up and running!";
    }

    // ROS 2
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_x_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_y_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_z_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    // MQTT
    mqtt::async_client client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<MqttRos2Bridge>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
