cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0009 NEW)
project(ultrasonic_control)

# ROS 2 Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mg400_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mqtt_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include-Verzeichnis hinzufügen
include_directories(include)

# Erstelle die ausführbare Datei für ultrasonic_control_node
add_executable(ultrasonic_control_node src/ultrasonic_control_node.cpp)

# Abhängigkeiten mit dem Target verbinden
ament_target_dependencies(ultrasonic_control_node 
  rclcpp
  std_msgs
  mg400_msgs
  std_srvs
  mqtt_bridge
  sensor_msgs
)

# Installation des ausführbaren Programms
install(
  TARGETS
    ultrasonic_control_node
  DESTINATION
    lib/${PROJECT_NAME}
)

# Header-Dateien mitinstallieren (optional, wenn du eigene Header hast)
install(
  DIRECTORY
    include/
  DESTINATION
    include
)

# Hier kommt die Installation des Launch-Ordners hinzu:
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2-Paket exportieren
ament_package()
