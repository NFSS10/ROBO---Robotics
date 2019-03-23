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

2nd - Next, setup the workspace so it's ready to be built once you get your package created:
```
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
```
3rd - Copy the package named "robo_assigment_ii" that is inside the "Packages" folder, to your workspace src folder "~/myrobo_ws/src"

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

---

## Conde Simulator
blabla
(Conde simulator repository for more information about the simulator and competition: https://github.com/ee09115/conde_simulator)  
(Old repository, ask for access if you want to see the steps made, by looking at the commits: https://gitlab.com/FEUPROBO/assigment-4---conde-robot.git)

### Software needed
ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu

Catkin Command Line Tools: https://catkin-tools.readthedocs.io/en/latest/index.html

### How to setup
1st - Create a catkin workspace:
```
mkdir -p ~/myrobo_ws/src
cd ~/myrobo_ws/src
```

2nd - Setup the workspace so it is ready to be built once you get your package created:
```
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
```

3rd - Copy all packages inside the provided "Packages" folder, to your workspace src folder "~/myrobo_ws/src"

4th - Compile the packages:
```
cd ~/myrobo_ws/
catkin_make
```

### How to run
1st - Open a terminal and open the Gazebo wo+rld:
```
cd ~/myrobo_ws/
source devel/setup.bash
roslaunch conde_world spawn_world.launch
```	

2nd - Open another terminal and spawn the robot:
```
cd ~/myrobo_ws/
source devel/setup.bash
roslaunch conde_world spawn_robot.launch
```

3rd - Open another terminal and start the conde_tracking node:
```
cd ~/myrobo_ws/
source devel/setup.bash
roslaunch conde_tracking run.launch
```

4th - Open another terminal and start the conde_signalling_panel node:
```
cd ~/myrobo_ws/
source devel/setup.bash
rosrun conde_signalling_panel conde_signalling_panel_node
```

5th - Open another terminal and start the conde_control node:
```
cd ~/myrobo_ws/
source devel/setup.bash
rosrun conde_control conde_control_node
```

6th - Open another terminal and start the conde_decision node:
```
cd ~/myrobo_ws/
source devel/setup.bash
rosrun conde_decision conde_decision_node
```

At this point, the robot is spawned in the world and all the required nodes are running.

7th - To control the signalling panel, open another terminal and start the gazebo_signalling_panel_control node:
```
cd ~/myrobo_ws/
source devel/setup.bash
rosrun gazebo_signalling_panel_control gazebo_signalling_panel_control_node

Use the numbers keys in your keyboard to change the signs.
[0] - left
[1] - right
[2] - up
[3] - stop
[4] - park
```

To spawn obstacles for the parking challenge open a terminal and use the following commands:
```
cd ~/myrobo_ws/
source devel/setup.bash
```
To spawn an obstacle in the parking spot closest to the crosswalk:
```
roslaunch conde_world spawn_parking_obstacles_v2.launch
```
To spawn an obstacle in the parking spot farthest to the crosswalk:
```
roslaunch conde_world spawn_parking_obstacles_v1.launch
```
