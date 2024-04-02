# docker build -t sebsti1/robot-autonomy .
FROM osrf/ros:humble-desktop-full-jammy

# add nano and gedit
RUN apt update &&\
    apt upgrade -y &&\
    apt install -y gedit nano ros-humble-turtlebot3* ros-humble-rqt-console ros-humble-rqt-tf-tree konsole

# create a user with id 1000 called ros2
RUN useradd ros2 -u 1000 -m -s /bin/bash &&\
    usermod -aG sudo ros2 &&\
    echo "ros2:ros2" | chpasswd

# create workspace and add source in bashrc
RUN mkdir /home/ros2/colcon_ws &&\
    cd /home/ros2/colcon_ws &&\
    colcon build &&\
    chown -R ros2:ros2 /home/ros2/colcon_ws &&\
    echo '\n# Add source for ros\n\
source /opt/ros/humble/setup.bash\n\
source /home/ros2/colcon_ws/install/setup.bash\n\n\
# Create alias to build workspace\n\
build() {\n\
  current_dir=$(pwd)\n\
  cd ~/colcon_ws\n\
  colcon build $@\n\
  source install/setup.bash\n\
  cd $current_dir\n\
}\n\n\
# build if package my_turtlebot doesn t exit\n\
ros2 pkg prefix my_turtlebot > /dev/null 2> /dev/null\n\
pkg_search_result=$?\n\
if [ $pkg_search_result == 1 ] ; then\n\
  echo "Can't find Rasmus' package, building the workspace."\n\
  build\n\
fi\n\
# Add the var to start the simulation\n\
export ROS_DOMAIN_ID=11\n\
export TURTLEBOT3_MODEL=burger\n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models\n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models' >> /home/ros2/.bashrc

# set the user as the used user
USER ros2
WORKDIR /home/ros2/colcon_ws