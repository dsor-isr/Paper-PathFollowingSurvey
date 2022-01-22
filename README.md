# Paper: Theory, simulations, and experiments of path following guidance strategies for autonomous robotic vehicles: Part I
This repository implements a simulation playground for performing path following experiments with the Medusa clas of marine vehicles. It implements the algorithms described in the survey paper "Theory, simulations, and experiments of path following guidance strategies for autonomous robotic vehicles: Part I".

<p align = "center">
<img src="img/Demo1.gif" width = "426" height = "240" border="5" />
</p>

### Ackowledgment
If you are using this code research and development for your publication, please cite:

```
@inproceedings{Hung_tuan,
	doi = {},
	url = {},
	year = 2022,
	month = {},
	publisher = {{IEEE}},
	author = {Hung Tuan and Francisco Rego and Joao Quintas and Joao Cruz and Marcelo Jacinto and Luis Sebastião and António Pascoal},
	title = {Theory, simulations, and experiments of path following guidance strategies for autonomous robotic vehicles: Part I},
	booktitle = {}
}
```

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

This repository assumes that you already have a machine running ubuntu 20.04LTS and a working installation of ROS1 (Noetic) and Gazebo 11. If you do not have ROS installed, please follow the steps available at the [ROS installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

### Install library requirements
- Install the Python requirements:
```
pip3 install --user numpy pandas matplotlib scipy sklearn rospkg catkin_pkg future joystick-controller
```

- Install C++ (apt-get) requirements:
```
sudo apt-get install python3-catkin-tools libgeographic-dev ros-noetic-geographic-msgs libxmlrcpp-dev librosconsole-dev libudev-dev libusb-1.0-0-dev ros-noetic-geodesy -y
```

- Install Geographiclib 1.50.1 (C++ library):
```
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
```

- Install Eigen version 3.4.0 (C++ equivalent of numpy in python):
```
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
```

### Setup a ROS workspace, clone the code and compile
```
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
```

### Run a simulation experiment
```
TODO
```

### License
This repository is open-sourced under the MIT license. See the [LICENSE](LICENSE) file for details.
