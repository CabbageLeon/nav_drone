#!/bin/bash
set -e
# 启动 PX4 v1.13 SITL 仿真环境
pkill micrortps_agent 2>/dev/null || true
cd /home/shuning/PX4-Autopilot && make px4_sitl gazebo
