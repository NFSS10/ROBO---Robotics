------------------------- Software needed to run: -------------------------
ROS: 
(we used kinectic)
	> http://wiki.ros.org/kinetic/Installation/Ubuntu

catkin tools:
	> https://catkin-tools.readthedocs.io/en/latest/index.html

turtlebot simulator:
(if you use kinetic as we did, you may have to change the keywords in the commands of the tutorial from indigo to kinetic)
	> http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation




------------------------- How to Setup: -------------------------
1st - Create a catkin workspace:
	> mkdir -p ~/myrobo_ws/src
	> cd ~/myrobo_ws/src

2nd - Next we'll setup the workspace so it is ready to be built once you get your package created:
	> source /opt/ros/kinetic/setup.bash
	> catkin_init_workspace

3rd - Copy our package named "robo_assigment_ii" that is inside the "Packages", to your workspace src folder "~/myrobo_ws/src"

4th - Compile our package:
	> cd ~/myrobo_ws/
	> catkin_make


------------------------- How to Run: -------------------------
After all the previous steps are done
1st - Open one terminal and open a gazebo world (note path may be different, the worlds files are in the "worlds" folder)
	To open the "V" shaped wall world:
	> roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/nuno/Desktop/ROBO_entrega/worlds/my_V_world
	
	To open the big, thick "W" world:
	> roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/nuno/Desktop/ROBO_entrega/worlds/my_W_world


2nd - Open another terminal and run our code:
	> source ~/myrobo_ws/devel/setup.bash
	> rosrun robo_assigment_ii robo_assigment_ii_node

