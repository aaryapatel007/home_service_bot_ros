# home_service_bot_ros

![Overview](https://github.com/aaryapatel007/home_service_bot_ros/blob/master/video/home-service.gif)  

## Project Aim :
1) Simulation setup for interfacing robot with different ROS packages, some of which are official ROS packages and others are packages that are created.

2) Map the environment of the Gazebo world with slam gmapping (Patience required).

3) Localization and Navigation testing where the robot is instructed to go to the pick up and drop off zones in the Gazebo world. Localization is done by AMCL and Navigation trajectory is done by ROS Navigation stack, which is based on Dijkstra's. 

4) Navigation Goal Node where a node is written that will communicate with the ROS Navigation stack and autonomously sending successive goals for the robot to reach. 

5) Simulating a Home service robot where the robot navigates to pick up and deliver virtual objects (using markers).

6) Writing shell scripts to execute home service simulation.

### Mapping  
Created a `test_slam.sh` script file and launched it to manually test SLAM.  
A functional map of the environment will be created which would be used for localization and navigation tasks.  
### Localization and Navigation  
Created a `test_slam_myrobot.sh` script file to launch it for manual navigation test.  
THe robot will be able to navigate in the environment after a 2D Nav Goal command is issued.  
Created a `pick_objects` node that will send multiple goals for the robot to reach.  
The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone."  
### Home Service Functions  
Created a `add_marker` node that will publish a marker to rviz.  
The marker will initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.
Finally wrote a `home_service.sh` file that will run all the nodes in this project.  
The home service robot will be simulated as follow:  
* Initially show the marker at the pickup zone.
* Hide the marker once your robot reach the pickup zone.
* Wait 5 seconds to simulate a pickup.
* Show the marker at the drop off zone once your robot reaches it.


## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Project Description  
Directory Structure  
```
.Home-Sevice-Robot                                        # Home Service Robot Project
├── catkin_ws                                             # Catkin workspace
│   ├── src
│   │   ├── add_markers                                   # add_markers package        
│   │   │   ├── launch
│   │   │   │   ├── add_markers.launch                    # launch file for add_markers node
│   │   │   ├── src
│   │   │   │   ├── add_markers.cpp                       # source code for add_markers node
│   │   ├── pick_objects                                  # pick_objects package   
│   │   │   ├── launch
│   │   │   │   ├── pick_objects.launch                   # launch file for pick_objects node
│   │   │   ├── src
│   │   │   │   ├── pick_objects.cpp                      # source code for pick_objects node
│   │   │   ├── maps                                      # maps folder for maps
│   │   │   │   ├── map.pgm
│   │   │   │   ├── map.yaml
│   │   ├── rvizConfig                                    # rvizConfig package        
│   │   │   ├── rvizConfig.rviz                           # rvizConfig file for home service robot   
│   │   ├── my_robot                                      # my_robot package        
│   │   │   ├── launch                                    # launch folder for launch files   
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── meshes                                    # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── urdf                                      # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                                    # world folder for world files
│   │   │   │   ├── myworld.world
│   │   │   ├── CMakeLists.txt                            # compiler instructions
│   │   │   ├── package.xml                               # package info
│   │   ├── shellScripts                                  # shell scripts files
│   │   │   ├── generate_maps.sh                          # shell script to generate maps  
│   │   │   ├── home_service.sh                           # shell script to launch home service robot   
│   │   │   ├── test_slam_myrobot.sh                      # shell script to test SLAM
│   │   │   ├── wall_follower.sh                          # shell script to launch wall follower node 
│   │   ├── wall_follower                                 # wall_follower node
│   │   │   ├── launch
│   │   │   │   ├── wall_follower.launch                  # launch file for wall_follower node
│   │   │   ├── src
│   │   │   │   ├── wall_follower.cpp                     # source code for wall_follower node
│   │   ├── video                                         
|   |   |   ├── home_service.gif                                 # GIF for overview
```
## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  

## Run the project  
* Clone this repository under the `catkin_ws/src` folder
```
git clone https://github.com/aaryapatel007/home_service_bot_ros.git
```
* Open the repository, make and source  
```
cd /home/workspace/catkin_ws/
catkin_make
source devel/setup.bash
```
* Launch the home service robot
```
./src/shellScripts/home_service.sh
```
* Done.  

## License

This repository is licensed under the terms of the MIT license.
