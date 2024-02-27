# Build:
# docker build -t seb-sti1/ros2-desktop-vnc:humble .
#
# Run:
# docker run -p 6080:80 --volume ./colcon_workspace:/workspace/src/external_workspace/ --security-opt seccomp=unconfined --shm-size=512m seb-sti1/ros2-desktop-vnc:humble

FROM tiryoh/ros2-desktop-vnc:humble

RUN apt update &&\
    apt install -y nano gedit ros-humble-turtlebot3*

RUN mkdir -p /workspace/src/external_workspace/ &&\
    cd /workspace/src/ &&\
    git clone https://github.com/RasmusAndersen/RobotAutonomy.git my_turtlebot &&\
    cd /workspace &&\
    colcon build

COPY ./setup_vars.sh /
RUN chmod +x /setup_vars.sh