#!/usr/bin/env bash

# -------------------------------
# Install the Python requirements
# -------------------------------
echo "Installing Python requirements"
pip3 install numpy matplotlib scipy sklearn rospkg catkin_pkg future joystick-controller

# ----------------------------------
# Install C++ (apt-get) requirements
# ----------------------------------
echo "Installing C++ requirements"
sudo apt-get -y install python3-catkin-tools libgeographic-dev ros-noetic-geographic-msgs librosconsole-dev libudev-dev libusb-1.0-0-dev ros-noetic-geodesy

# Install Geographiclib 1.50.1 (C++ library):
wget -q https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.50.1.tar.gz/download && \
tar xfpz download && \
cd GeographicLib-1.50.1 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R download GeographicLib-1.50.1

# Install Eigen version 3.4.0 (C++ equivalent of numpy in python):
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
tar xfpz eigen-3.4.0.tar.gz && \
cd eigen-3.4.0 && \
mkdir BUILD && \
cd BUILD && \
cmake .. && \
sudo make && \
sudo make install && \
cd .. && \
cd .. && \
sudo rm -R eigen-3.4.0 eigen-3.4.0.tar.gz

# -----------------------------------------------------
# Setup a ROS workspace, clone the code and compile
# -----------------------------------------------------
echo "Setting up caktin workspace"

# Run the following lines to add elements to the .bashrc file
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export CATKIN_ROOT=${HOME}" >> ~/.bashrc
echo "export ROS_WORKSPACE=${CATKIN_ROOT}/catkin_ws" >> ~/.bashrc

# Go to the home folder
cd ${HOME}

# Create a catkin workspace
mkdir catkin_ws

# Go to the catkin workspace and clone this repository to a src folder
cd catkin_ws
git clone --recursive https://github.com/dsor-isr/Paper-PathFollowingSurvey.git src

# Run the following lines to add elements to the .bashrc file
echo "export FAROL_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol_scripts | head -n 1)" >> ~/.bashrc
echo "source ${ROS_WORKSPACE}/devel/setup.bash" >> ~/.bashrc

# Source the most up to data bashrc file
source ~/.bashrc

# Compile the code using catkin build
catkin build