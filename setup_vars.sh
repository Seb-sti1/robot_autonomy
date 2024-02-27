#!/bin/bash

echo "source /workspace/install/setup.bash

export ROS_DOMAIN_ID=11
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
" >> /home/ubuntu/.bashrc