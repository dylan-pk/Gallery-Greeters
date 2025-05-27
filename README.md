# Gallery-Greeters
Gallery Greeters is an automatic robotic solution being developed for the Tom Ugly’s Art Gallery’s opening art exhibition.
The solution will be part of a three-stage system, the Gallery Greeters project will handle the hosting role of the gallery.

## Table of Contents

## Project Overview
### Key System Features
* Greeting Guests at the Doorway
* Leading Guests to Available Tables
* Placing Drink Orders
* Leading Guests to the Artworks
* Wander the Gallery waiting to Assist Guests
* Entertaining Guests with Art Based Fun Facts, Information about Artworks or completing a “Dance”
* Returning to its Charging Station
* Providing Information about Table Capacity and Order Status

### The Subsystems
1. ***Voice Commands***, which handle the direct interaction between the robot and users.
2. ***Path Planning***, which sets safe paths for the robot to travel around the environment.
3. ***Control***, which handles the velocity commands of the robot as well as obstacle avoidance.
4. ***Perception***, which is used to identify the artworks within the environment.

## Requirements

| Hardware | Software | Libraries |
|----------|----------|-----------|
| TurtleBot3 (w/ camera, lidar and external connectivity access) | Ubuntu 22.04 | Python |
| Microphone | ROS2 | Porcupine from PicoVoice |
| Digital Screen | Gazebo | Google Speech to Text |
| | VSCode | Pyttsx3 |
| | | Pyaudio |
| | | Pillow |
| | | Screeninfo |
| | | Sounddevice |
| | | Opencv-python|
| | | CV_Bridge |
| | | Numpy |

## Installation
### Ros2 Humble Install Instructions
Ensure that the Ubuntu Universe repository is enabled
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add the ROS 2 GPG key
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o
/usr/share/keyrings/ros-archive-keyring.gpg
```
Add the repository to your sources list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-
keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo
$UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/nul
```
Update your apt repository caches after setting up the repositories and ensure your system is up
to date before installing new packages
```
sudo apt update
sudo apt upgrade
```
Install the desktop, ROS-Base and development tools
```
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```
Source the following file to setup the environment
```
source /opt/ros/humble/setup.bash
```

### Clone the Gallery Greeters Repository
If you don’t have a ros2 work space directory already
```
mkdir -p ~/ros2_ws/src
```
once you have that directory
```
cd ~/ros2_ws/src
```
Clone the Gallery Greeters repository via SSH
```
git clone git@github.com:dylan-pk/Gallery-Greeters.git
```
OR (if no SSH key is set up)
```
git clone https://github.com/dylan-pk/Gallery-Greeters.git
```
### Install External Libraries
```
pip install pvporcupine SpeechRecognition pyttsx3 pyaudio Pillow screeninfo sounddevice
opencv-python numpy
```
### Install ROS Bridge Pack
```
sudo apt install ros-humble-cv-bridge
```
### Build and Source the Workspace
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

