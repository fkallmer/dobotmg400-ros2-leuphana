cmake_minimum_required(VERSION 3.5)
project(mqtt_bridge)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)      # Hinzugefügt: std_srvs
find_package(PahoMqttCpp REQUIRED)

# Create executable
add_executable(mqtt_node src/mqtt_node.cpp)
ament_target_dependencies(mqtt_node rclcpp std_msgs std_srvs sensor_msgs PahoMqttCpp)

target_link_libraries(mqtt_node PahoMqttCpp::paho-mqttpp3)

install(TARGETS
  mqtt_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
