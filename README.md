# racecar

Gazebo 3D simulator example of ROS tutorials (version: Noetic)

Tested on Ubuntu 20.04

For more detail, see https://www.bilibili.com/video/BV1da411j72x

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

![1](https://user-images.githubusercontent.com/26653172/170944302-b99881db-936f-4168-bba8-6378350f2b26.png)

You should download the gazebo models or the map won't work.

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

5. Open a new terminal and start rviz
```bash
roslaunch racecar_gazebo slam_gmapping.launch 
```

Control the car with WASD keys and draw the whole map.

![2](https://user-images.githubusercontent.com/26653172/170944385-2b7a32f2-0e87-47d5-8b5c-52874e1bd9e4.png)

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

![3](https://user-images.githubusercontent.com/26653172/170944487-3984d093-f43e-429a-95ac-0cd4114e5fc1.png)

3. Click `2D Nav Goal` then click the goal on the rviz map (hold down the cursor to set the goal pose)

4. Start the navigation script
```bash
rosrun racecar_gazebo path_pursuit.py
```

![4](https://user-images.githubusercontent.com/26653172/170944551-d4fa12c8-7ea1-4af8-9c7a-bd75deb8a7ea.png)

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
