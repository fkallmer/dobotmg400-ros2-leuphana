#!/bin/bash
xhost +local:
docker compose up -d
docker exec -it ros2_humble_full bash