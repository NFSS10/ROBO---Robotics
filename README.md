# ROBO - Robotics
Projects made in my Robotics class - MIEIC 5y1s

## Reactive Robot
A subsumption reactive robot using ROS (Robot Operating System) programmed to follow the outside of a "V" shaped wall and the interior walls of a large and very thick "W" shaped map, utilizing the turtlebot and gazebo simulation.  

### Software needed
ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu  
(Other version should work too, but ROS Kinetic was the one used in the development)

Catkin Command Line Tools: https://catkin-tools.readthedocs.io/en/latest/index.html

Turtlebot Simulator: http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation  
(If you use ROS Kinetic as I did, you may have to change the keywords in the commands of the tutorial from indigo to kinetic)

### How to setup
1st - Create a catkin workspace:
```
mkdir -p ~/myrobo_ws/src
cd ~/myrobo_ws/src
```

2nd - Next setup the workspace so it's ready to be built once you get your package created:
```
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
```
3rd - Copy my package named "robo_assigment_ii" that is inside the "Packages" folder, to your workspace src folder "~/myrobo_ws/src"

4th - Compile the package:
```
cd ~/myrobo_ws/
catkin_make
```

### How to run
1st - Open one terminal and open a Gazebo world (note path may be different, the worlds files are in the "worlds" folder)  
To open the "V" shaped wall world:
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/your_path_here/worlds/my_V_world
```
To open the big, thick "W" world:
```
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/your_path_here/worlds/my_W_world
```


2nd - Open another terminal and run the code:
```
source ~/myrobo_ws/devel/setup.bash
rosrun robo_assigment_ii robo_assigment_ii_node
```
