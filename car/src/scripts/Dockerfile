# This is an auto generated Dockerfile for ros:perception
# generated from docker_images_ros2# FROM ros:humble-perception-jammy
FROM ros:humble-perception
# FROM osrf
SHELL ["
# Custom CMake install to get v3.28
ENV CMAKE_VERSION=3.28.1
RUN apt-get remove -y cmake
RUN apt-get update && \
    apt-get -y install build-essential wget
RUN rm -rf RUN wget https:      -q -O RUN wget -O - https:RUN echo 'deb [signed-by=RUN chmod u+x RUN mkdir RUN RUN rm RUN ln -s 

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop* \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-irobot-create-msgs \
    ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    ros-humble-tf-transformations python3-rosdep2 \
    nano emacs joystick\
    python3-pip \
    && rm -rf 

   
RUN apt update && apt-get install -y sudo python3-pip software-properties-common lsb-release make protobuf-compiler procps

#humble packages
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ffmpeg

# ros2 packages
RUN sudo apt-get update && apt-get install -y sudo python3-pip ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard unzip
RUN sudo apt-get install -y sudo xterm clinfo locate lsb-release wget gnupg inetutils-ping net-tools ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ur ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-librealsense2* ros-humble-realsense2*
RUN sudo updatedb

ARG USER=dock
ARG PASSWD=dock

RUN useradd -d 
RUN usermod -aG dialout dock

RUN mkdir -p 
RUN chown -R dock:dock 20xt_ws

RUN echo "dock ALL=(ALL) NOPASSWD: ALL" >> 
USER dock



RUN touch ~
RUN chmod -R +rwx 20xt_ws

RUN bash     cd     rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

RUN bash     cd     git clone -b $ROS_DISTRO https:    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y
    


RUN bash -c 'source     cd     colcon build && \
    ls     source     ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    source 
ADD src
RUN bash -c 'source     cd     colcon build'


RUN echo 'source 

WORKDIR 
