#!/usr/bin/env bash

# Install the Python requirements:
pip3 install --user numpy pandas matplotlib scipy sklearn rospkg catkin_pkg future joystick-controller

# Install C++ (apt-get) requirements:
sudo apt-get install python3-catkin-tools libgeographic-dev ros-noetic-geographic-msgs libxmlrcpp-dev librosconsole-dev libudev-dev libusb-1.0-0-dev ros-noetic-geodesy -y

# Install Geographiclib 1.50.1 (C++ library):
wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.50.1.tar.gz/download
tar xfpz download
cd GeographicLib-1.50.1 
mkdir BUILD
cd BUILD
cmake ..
sudo make
sudo make test
sudo make install
cd ..
cd ..
sudo rm -R download GeographicLib-1.50.1

# Install Eigen version 3.4.0 (C++ equivalent of numpy in python):
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar xfpz eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir BUILD
cd BUILD
cmake ..
sudo make
sudo make install
cd ..
cd ..
sudo rm -R eigen-3.4.0 eigen-3.4.0.tar.gz

# -----------------------------------------------------
# Setup a ROS workspace, clone the code and compile
# -----------------------------------------------------

# Run the following lines to add elemnts to the .bashrc file
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export CATKIN_ROOT=${HOME}" >> ~/.bashrc
echo "export ROS_WORKSPACE=${CATKIN_ROOT}/catkin_ws" >> ~/.bashrc
echo "export MEDUSA_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname medusa_scripts | head -n 1)" >> ~/.bashrc
echo "source ${ROS_WORKSPACE}/devel/setup.bash" >> ~/.bashrc

# Go to the home folder
cd ~/

# Create a catkin workspace
mkdir catkin_ws

# Go to the catkin workspace and clone this repository to a src folder
cd catkin_ws
git clone --recursive git@github.com:dsor-isr/medusa_base.git src

# Source the most up to data bashrc file
source ~/.bashrc

# Compile the code using catkin build
catkin build