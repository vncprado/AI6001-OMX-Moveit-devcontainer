# disable the cached network settings in the web-browser, ctrl+shift+i -> network -> disable cache. 

# password -> vscode

# build the image with:
# docker build -t ubuntu-docker .

# run the image with:
# docker run -it -p 6080:6080 --name ubuntu-container ubuntu-docker

# FROM ubuntu:20.04
FROM osrf/ros:noetic-desktop-full

# prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# update dependencies
RUN apt update
RUN apt upgrade -y

# install dependencies
RUN apt install -y \
  net-tools \
  nano \
  vim \
  neovim \
  curl \
  wget \
  firefox \
  git \
  python3 \
  python3-pip

#install moveit and other dependencies for the OpenManipulator Project
RUN apt install -y ros-noetic-robotis-manipulator ros-noetic-dynamixel-* ros-noetic-open-manipulator-* ros-noetic-moveit* ros-noetic-joint-trajectory-controller

#install pcl
RUN apt install -y ros-noetic-pcl-ros pcl-tools python3-pcl ros-pcl-msgs

#install openCV
RUN apt install -y libopencv-dev python3-opencv

WORKDIR /root

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "/bin/bash", "-c"]