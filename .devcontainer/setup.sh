#!/bin/bash

# ROS2 환경 설정
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=13' >> ~/.bashrc
echo 'alias sb="source ~/.bashrc && echo \"bashrc is reloaded\""' >> ~/.bashrc

# 환경설정 적용
source ~/.bashrc

# 워크스페이스 빌드 및 환경 설정
cd ~/ros2-ws
colcon build

# 빌드 후 환경 설정
echo 'source ~/ros2-ws/install/local_setup.bash' >> ~/.bashrc
source ~/.bashrc

echo "ROS2 Humble 환경 설정 완료"
