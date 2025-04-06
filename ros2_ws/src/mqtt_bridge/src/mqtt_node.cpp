/*
© 2025 Falk-Richard Kallmer, Leuphana University Lüneburg
MIT License – see LICENSE file for details
Contact: falk-richard.kallmer@stud.leuphana.de

This script initializes the MQTT-ROS2 bridge for integrating ultrasonic sensor published by the ESP32 data into a ROS2 system.
It subscribes to MQTT topics, processes incoming messages, and publishes them as ROS2 Range messages.
*/

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <memory>
#include "sensor_msgs/msg/range.hpp"

// Alias for JSON library
using json = nlohmann::json;

// MQTT broker address and client ID
const std::string ADDRESS = "tcp://localhost:1883";
const std::string CLIENT_ID = "mqtt_bridge";

// List of MQTT topics to subscribe to
const std::vector<std::string> MQTT_TOPICS = {
    "sensor/distance/x",
    "sensor/distance/y",
    "sensor/distance/z"
};

// ROS2 Node class that bridges MQTT messages to ROS2 topics
class MqttRos2Bridge : public rclcpp::Node, public mqtt::callback
{
public:
    MqttRos2Bridge()
    : Node("mqtt_bridge"), // Initialize ROS2 node with the name "mqtt_bridge"
      client_(ADDRESS, CLIENT_ID) // Initialize MQTT client
    {
        // Create ROS2 publishers for ultrasonic sensor data
        pub_x_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic/x", 10);
        pub_y_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic/y", 10);
        pub_z_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic/z", 10);

        // Create a ROS2 service to check the status of the MQTT bridge
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/mqtt_bridge/mqtt_node",
            std::bind(&MqttRos2Bridge::handle_mqtt_bridge_service, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Set MQTT callback to this class
        client_.set_callback(*this);

        // Configure MQTT connection options
        mqtt::connect_options conn_opts;
        conn_opts.set_keep_alive_interval(10); // Keep-alive interval in seconds
        conn_opts.set_clean_session(true); // Clean session on reconnect

        try {
            // Connect to the MQTT broker
            client_.connect(conn_opts)->wait();
            RCLCPP_INFO(this->get_logger(), "Erfolgreich mit MQTT-Broker bei %s verbunden.", ADDRESS.c_str());
        }
        catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Verbinden mit MQTT-Broker: %s", e.what());
        }

        // Subscribe to the specified MQTT topics
        for (const auto &topic : MQTT_TOPICS) {
            client_.subscribe(topic, 0);
            RCLCPP_INFO(this->get_logger(), "Auf MQTT-Topic abonniert: %s", topic.c_str());
        }
    }

    // Callback for when an MQTT message arrives
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        try {
            std::string payload = msg->to_string(); // Get the message payload
            if (payload.empty()) {
                return; // Ignore empty messages
            }

            // Parse the payload as JSON
            json j = json::parse(payload);

            // Create a ROS2 Range message
            sensor_msgs::msg::Range range_msg;
            range_msg.header.stamp = this->now(); // Set the timestamp

            // Set common properties for the Range message
            range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            range_msg.field_of_view = 0.5f;
            range_msg.min_range = 0.02f;
            range_msg.max_range = 4.0f;

            // Publish the message to the appropriate ROS2 topic based on the MQTT topic
            if (msg->get_topic() == "sensor/distance/x" && j.contains("x")) {
                range_msg.range = j["x"].get<float>() * 0.01f; // Convert to meters
                range_msg.header.frame_id = "ultrasonic_x";
                pub_x_->publish(range_msg);

            } else if (msg->get_topic() == "sensor/distance/y" && j.contains("y")) {
                range_msg.range = j["y"].get<float>() * 0.01f; // Convert to meters
                range_msg.header.frame_id = "ultrasonic_y";
                pub_y_->publish(range_msg);

            } else if (msg->get_topic() == "sensor/distance/z" && j.contains("z")) {
                range_msg.range = j["z"].get<float>() * 0.01f; // Convert to meters
                range_msg.header.frame_id = "ultrasonic_z";
                pub_z_->publish(range_msg);

            } else {
                // Log a warning if the expected key is missing
                RCLCPP_WARN(this->get_logger(),
                    "Empfangene MQTT-Nachricht auf %s, aber erwarteter Schlüssel fehlte.",
                    msg->get_topic().c_str());
            }
        }
        catch (const json::parse_error &e) {
            // Log an error if JSON parsing fails
            RCLCPP_ERROR(this->get_logger(), "JSON-Parsing-Fehler: %s", e.what());
        }
        catch (const std::exception &e) {
            // Log any other exceptions
            RCLCPP_ERROR(this->get_logger(), "Fehler bei der Verarbeitung der MQTT-Nachricht: %s", e.what());
        }
    }

private:
    // Service callback to handle requests to check the MQTT bridge status
    void handle_mqtt_bridge_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service /mqtt_bridge/mqtt_node wurde aufgerufen.");
        response->success = true; // Indicate success
        response->message = "MQTT Bridge ist aktiv."; // Provide a status message
    }

    // ROS2 publishers for ultrasonic sensor data
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_x_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_y_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_z_;

    // ROS2 service for checking the MQTT bridge status
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    // MQTT client for communication with the broker
    mqtt::async_client client_;
};

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create an instance of the MqttRos2Bridge node
    auto node
    rclcpp::shutdown();
    return 0;
}
