# Dieses Skript startet die Docker-Umgebung für ROS 2 Humble

# Erlaubt Zugriff auf den X-Server für grafische Anwendungen
xhost +local:
# Startet die Docker-Container im Hintergrund
docker compose up -d
# Öffnet eine Bash-Shell im Container 'ros2_humble_full', um mit ROS 2 zu arbeiten
docker exec -it ros2_humble_full bash