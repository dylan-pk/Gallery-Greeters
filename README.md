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
## Pre-Running Code - API Key, Path Directory and Data Update
Please note that at the current stage of development, before running the code your own api key for access to Porcupine from PicoVoice requires your own API key which can be obtained from the [PicoVoice Console](https://console.picovoice.ai/). Simply create an account and request a porcupine API key, then copy it from the website and replace the current ACCESS KEY variable in SpeechToText.py with your access key.

Secondly the path directory must be updated in the artbot.launch.py file replacing user in line with your Linux user name and for the real map replacing the real_map_name with your map
```
sim_map_file = os.path.expanduser('/home/<USER>/ros2_ws/gallery_map.yaml') 
real_map_file = os.path.expanduser('/home/<USER>/ros2_ws/<real_map_name>.yaml')
```
## Running The System - Simulation
### Clean Gazebo and Nav2
To ensure the packages and software launches correctly, kill any Gazebo or Rvis processes using the commands below. Doing so will allow the map to launch freshly each time and ensure correct localisation.
```
killall -9 gzserver gzclient  
pkill –f map_server  
pkill –f nav2  
pkill –f rviz2
```
### Run the Gallery Greeters Launch File 
Launching the artbot.launch.py file using the commands below will open up the Gazebo world with the Turtlebot spawned into its charging port. It will load the map and then run the control, path planning and perception packages. All will be active and ready to interact when voice commands are given and the path planning package will automatically localise the robot within the map so it is ready to plan it’s path. The live occupancy grid will also open showing a black background of free space and the known objects in white which will update as new objects are detected.
```
ros2 launch gallery_greeters2 artbot.launch.py use_simulation:=true
```
![Expected Screens after successful launch](/ReadMeImages/LaunchFileScreenshot.png)

## Running The System - Real Robot
### Setting Up the Environment
The first step to running the Gallery Greeters artbot in a real environment is to set up the physical environment. Within the walls of the gallery, mark out the starting position, known obstacles/tables and the artworks along the gallery walls like seen below.

![Example of a Turtlebot in a Real Example Environment](/ReadMeImages/mapping_evidence.jpeg)

### Connecting to The Turtlebot
To establish a connection to the TurtleBot your computer must be connected to the same network, for example our testing was connected to a network called TurtleBot MMR.

Once both the TurtleBot and your computer are connected to the same network, connect with it via SSH. This requires knowing the IP address of the TurtleBot if this is unknown boot into the TurtleBot system with a monitor and keyboard and run the following command which will return your IP address in the terminal and note the username of the pi for the SSH connection.
```
hostname -I
```
Set the robot up into the starting position of the environment and connect the battery to power on in the home position.  

Ping the IP address until it returns that it is active. 
```
ping <IP address>
```
Once the robot is able to be connected with then ssh into the raspberry pi. For example `ssh ubuntu@192.168.0.213`.
```
ssh <pi username>@<IP address> 
```
Then enter the password of the TurtleBot

Launch the robot using a unique domain ID. For example `export ROS_DOMAIN_ID = 44`
```
export  ROS_DOMAIN_ID = <ID>
ros2 launch turtlebot3_bringup robot.launch.py 
```
Repeat the SSH process in another terminal to activate the camera on the TurtleBot, instead of the robot launch command run this command. Ensure you use the same unique Domain ID
```
export ROS_DOMAIN_ID = <ID> 
ros2 run camera_ros camera_node --ros-args -p format:=MJPEG -p width:=640 -p height:=480
```
### Mapping the Environment
With the SSH and TurtleBot active now run through the mapping process as outlined below **running each command in a new terminal**. 
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True 
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz 
ros2 run turtlebot3_teleop teleop_keyboard 
ros2 run nav2_map_server map_saver_cli -f <map name>
```
Below is an example of the map generated with the physical environment from before. 

![Real World Map Example](/ReadMeImages/real_map5.png)

Now reset the TurtleBot to its starting position severing the connection with the computer and powering it back on in the starting position. Teleop the robot to each of the artworks and tables gathering the coordinates of each known position and orientation keeping the robot far away enough from the artworks so that they are in frame of the camera, this is roughly 30cm away from the artwork when using A4 paper. 

Then update the artwork locations within package_audio_messages using the following command and input the names of the artworks along with the locations and orientation that the robot needs to be in for the camera to percieve the artwork. 
```
cd ros2_ws/src/gallery_greeters/package_audio_messages/package_audio_messages/ 
python3 ArtDataInput.py
```
### Run the Physical ArtBot
Rerun the connection to the TurtleBot with the robot powered on in the home position. Then in a new terminal run the launch file.
```
export ROS_DOMAIN_ID =<ID> 
ros2 launch gallery_greeters2 artbot.launch.py use_simulation:=false
```
## Interacting with The Robot
_Bolded words are the key words which trigger commands_
To initialise the interaction start by calling out to the robot “Hey **Jarvis**” 

There are 10 ways to interact with Jarvis as an operator or guest of the art gallery: 

1. To tell ArtBot to go greet guests say “Go into **Greeting** mode”
    - To which the ArtBot will move to his set position for greeting guests
    
![Artbot in Greeting Mode](/ReadMeImages/Interaction1.png)
    
2. To place a drinks order with Artbot say “I would like a **drink**”
    - The Artbot will respond asking what drink you would like displaying the options for you.
    - Artbot will recognise the type of drink you ask for if it is on the list and follow up asking for the quantity of drinks you would like.
    - After confirming these details, Artbot will ask if you want any more drinks until you say no.
    - The order will then be sent to the kitchen and wait staff robots.

![Drink Options Screen](/ReadMeImages/Interaction2.png)

3. To ask about the art say “I would like to look at the **art**” or “Tell me about the **artwork**"
    - To which Jarvis will ask “Which Artwork would you like to know about”
    - Respond with a simplified name such as “the one with clocks”, “The one from Minecraft” or “The Duchess” and the artbot will take you to it.

![ArtBot Positioned infront of Artwork for Scanning](/ReadMeImages/Interaction3.png)

4. To put Jarvis in Sentry Mode (Moving around the environment waiting for guest interaction) say “Go for a **walk**”
    - To which Jarvis will plan a path around the perimeter of the gallery and continue that path until another “Hey Jarvis” command is given.

![ArtBot Wandering Parimeter of Environment](/ReadMeImages/Interaction4.png)

5. To get Jarvis to dance say “Do a **Dance**”
    - To which Jarvis will dance on the spot

6. To send Jarvis back to the charging port say “Go and **charge**”
     - To which Jarvis will go to his Initial position at his charging port.

![ArtBot in Charging Mode](/ReadMeImages/Interaction5.png)

7. To get Jarvis to take you to a table say “Take me to a **Table**”
     - To which Jarvis will ask which table and after giving him a number from 1-3 he will move to one of the 3 available tables.

![ArtBot Going to a Table](/ReadMeImages/Interaction6.png)

8. To call a waiter to the table say “Call a **waiter**” which will bring a waiter to the closest table

9. To ask Jarvis for a fun fact say “Tell me a fun **fact**”
      - To which Jarvis will respond with one of his many art based fun facts
      - 
![Van Gough Fun Fact](/ReadMeImages/Interaction7.png)

10. To receive an update about the status of the tables say “Tell me the **table status**”
      - Jarvis will then display an infographic visually depicting the status of all tables
  
![Example Table Status](/ReadMeImages/Interaction8.png)

11. To be taken through each artwork, ask Jarvis ”Take me to a **Painting**” and Jarvis will take you to an artwork without you needing to ask which artwork it

## Troubleshooting and FAQs
### Issues Installing pyaudio
If there are build issues installing pyaudio try running the following command
```
sudo apt-get install portaudio19-dev python3-pyaudio
```
### Map Not Showing Up in Rvis 
Make sure you have corrected the file path on line 20 with your linux user name
```
map_file = os.path.expanduser('/home/<USER>/ros2_ws/gallery_map.yaml')
```
Also ensure that you have killed all Gazebo and Rvis processes before running the launch file 
```
killall -9 gzserver gzclient  
pkill –f map_server  
pkill –f nav2  
pkill –f rviz2
```
### RQT Image View Issues 
When running the template matching launch file and attempting to view image data through the rqt_image_view window the data isn’t displayed. This issue is bypassed by running the following command. This will continue to open the latest frame if the defualt image viewer app is set to the built in linux image viewer. 
```
xdg-open /tmp/latest_camera_frame.jpg
```
