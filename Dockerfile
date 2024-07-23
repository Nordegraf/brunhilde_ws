# Docker Image for development of solo8 software
FROM ubuntu:24.04

ARG ROS_DISTRO=jazzy

ENV XDG_RUNTIME_DIR=/tmp/runtime-root

### ROS Setup ###
# locales
RUN apt-get update
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt-get -y install apt-utils
RUN sh -c "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"
RUN apt-get install -y -q

# universe
RUN apt-get install -y software-properties-common curl git
RUN add-apt-repository universe

# ROS GPG key
RUN apt-get update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS repositories
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
RUN apt-get update && apt-get upgrade -y

# ROS dev tools
RUN apt-get install -y ros-dev-tools
RUN apt-get install -y ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro
RUN apt-get install -y ros-$ROS_DISTRO-ros-gz
RUN apt-get install -y ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-gz-ros2-control ros-$ROS_DISTRO-imu-tools
RUN apt-get install -y python3 python3-pip python3-colcon-common-extensions python3-colcon-mixin python3-numpy python-is-python3 python3-vcstool
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

WORKDIR "/home/brunhilde_ws"

### Solo8 Setup ###
COPY src /home/brunhilde_ws/src
RUN apt-get install -y libeigen3-dev pybind11-dev doxygen ros-jazzy-eigenpy libyaml-cpp-dev libboost-all-dev

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

RUN echo "source /home/brunhilde_ws/install/setup.bash" >> /root/.bashrc

ENV DISPLAY=host.docker.internal:0.0