#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>  // For Trigger service
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <memory>
#include "sensor_msgs/msg/range.hpp"

using json = nlohmann::json;

// MQTT broker address & topics
const std::string ADDRESS = "tcp://localhost:1883";
const std::string CLIENT_ID = "mqtt_bridge";
// Du kannst diese MQTT-Themen beibehalten, auch wenn du ROS-Topics änderst
const std::vector<std::string> MQTT_TOPICS = {
    "sensor/distance/x",
    "sensor/distance/y",
    "sensor/distance/z"
};

class MqttRos2Bridge : public rclcpp::Node, public mqtt::callback
{
public:
    MqttRos2Bridge()
    : Node("mqtt_bridge"),
      client_(ADDRESS, CLIENT_ID)
    {
        // --------------------------------------------------
        // ROS 2 Publisher: Range (anstelle von Float32)
        // --------------------------------------------------
        // Hier drei Publisher für X, Y und Z:
        pub_x_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic/x", 10);
        pub_y_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic/y", 10);
        pub_z_ = this->create_publisher<sensor_msgs::msg::Range>("/sensor/ultrasonic/z", 10);

        // --------------------------------------------------
        // ROS 2 Service: /mqtt_bridge/mqtt_node
        // --------------------------------------------------
        // Erlaubt z.B. anderen Nodes zu checken, ob der Bridge-Node läuft
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/mqtt_bridge/mqtt_node",
            std::bind(&MqttRos2Bridge::handle_mqtt_bridge_service, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // --------------------------------------------------
        // MQTT Setup
        // --------------------------------------------------
        client_.set_callback(*this);
        mqtt::connect_options conn_opts;
        conn_opts.set_keep_alive_interval(10);
        conn_opts.set_clean_session(true);

        try {
            client_.connect(conn_opts)->wait();
            RCLCPP_INFO(this->get_logger(), "Erfolgreich mit MQTT-Broker bei %s verbunden.", ADDRESS.c_str());
        }
        catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Verbinden mit MQTT-Broker: %s", e.what());
        }

        for (const auto &topic : MQTT_TOPICS) {
            client_.subscribe(topic, 0);
            RCLCPP_INFO(this->get_logger(), "Auf MQTT-Topic abonniert: %s", topic.c_str());
        }
    }

    // --------------------------------------------------
    // MQTT callback für ankommende Nachrichten
    // --------------------------------------------------
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        try {
            std::string payload = msg->to_string();
            if (payload.empty()) {
                return;  // leere Nachricht ignorieren
            }

            // JSON parsen
            json j = json::parse(payload);

            // Gemeinsame Range-Nachricht vorbereiten
            sensor_msgs::msg::Range range_msg;
            range_msg.header.stamp = this->now();
            // z.B. "ultraschall_link" oder passend zum echten Sensor
            range_msg.header.frame_id = "ultraschall_link";

            // Typische Felder für Ultraschall
            range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            // Beispielhafte Werte: bitte an deine Sensor-Spezifikationen anpassen
            range_msg.field_of_view = 0.5f;   // ~28.6° Öffnungswinkel
            range_msg.min_range = 0.02f;      // z.B. 2 cm
            range_msg.max_range = 3.0f;       // z.B. 4 Meter

            // Nun prüfen wir, auf welchem MQTT-Topic wir die Daten bekommen haben
            // und setzen range_msg.range entsprechend.
            if (msg->get_topic() == "sensor/distance/x" && j.contains("x")) {
                range_msg.range = j["x"].get<float>();
                pub_x_->publish(range_msg);

            } else if (msg->get_topic() == "sensor/distance/y" && j.contains("y")) {
                range_msg.range = j["y"].get<float>();
                pub_y_->publish(range_msg);

            } else if (msg->get_topic() == "sensor/distance/z" && j.contains("z")) {
                range_msg.range = j["z"].get<float>();
                pub_z_->publish(range_msg);

            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Empfangene MQTT-Nachricht auf %s, aber erwarteter Schlüssel fehlte.",
                    msg->get_topic().c_str());
            }
        }
        catch (const json::parse_error &e) {
            RCLCPP_ERROR(this->get_logger(), "JSON-Parsing-Fehler: %s", e.what());
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler bei der Verarbeitung der MQTT-Nachricht: %s", e.what());
        }
    }

private:
    // --------------------------------------------------
    // Service-Callback
    // --------------------------------------------------
    void handle_mqtt_bridge_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service /mqtt_bridge/mqtt_node wurde aufgerufen.");
        response->success = true;
        response->message = "MQTT Bridge ist aktiv.";
    }

    // ROS 2: Publisher (Range statt Float32)
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_x_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_y_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_z_;

    // ROS 2: Service
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
