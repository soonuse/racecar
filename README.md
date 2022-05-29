# racecar

Gazebo 3D simulator example of ROS tutorials (version: Noetic)

Tested on Ubuntu 20.04

## Dependencies

```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control
sudo apt install ros-$ROS_DISTRO-effort-controllers
sudo apt install ros-$ROS_DISTRO-joint-state-controller
sudo apt install ros-$ROS_DISTRO-driver-base
sudo apt install ros-$ROS_DISTRO-ackermann-msgs
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
sudo apt install ros-$ROS_DISTRO-teb-local-planner

sudo apt install tcl-dev tk-dev python3-tk
```

## Getting Started

### Save the world map

1. Create a project workspace
```bash
mkdir -p ~/racecar_ws/src
```

2. git clone and compile it
```bash
cd ~/racecar_ws/src
git clone https://github.com/soonuse/racecar.git
cd ..
catkin_make
```

3. Setup .bashrc
```bash
echo "source ~/racecar_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. Start the gazebo map

```bash
roslaunch racecar_gazebo racecar_runway.launch
```

You should download the gazebo models or the map won't work.

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

5. Open a new terminal and start rviz
```bash
roslaunch racecar_gazebo slam_gmapping.launch 
```
Control the car with WASD keys and draw the whole map.

6. Save the map
```bash
rosrun map_server map_saver -f ~/racecar_ws/src/racecar/racecar_gazebo/map/map_runway
```

### Path planning and automatic navigation

1. Start navigation and the gazebo map

```bash
roslaunch racecar_gazebo racecar_runway_navigation.launch
```

2. Start rviz
```bash
roslaunch racecar_gazebo racecar_rviz.launch
```

3. Click `2D Nav Goal` then click the goal on the rviz map (hold down the cursor to set the goal pose)

4. Start the navigation script
```bash
rosrun racecar_gazebo path_pursuit.py
```

## FAQ

### Error: opencv not found

1. find OpenCVConfig.cmake
```bash
sudo apt install locate
sudo updatedb
locate OpenCVConfig.cmake
```

2. edit CMakeLists.txt and replace OpenCV_DIR with the correct OpenCV path
replace this line
```
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)
```
with the correct OpenCV path
