# msc_tt3_explore

This is a ROS package which uses the Navigation stack for path planning and Gmapping for SLAM to autonomously explore an previously unknown virtual environment with one or more TurtleBot3 robots, outputting occupancy grids of the explored environment. The Gazebo simulator is used for the simulation of the Turtlebot3 Waffle Pi robot. Various algorithms have been integrated for the Autonomously exploring the region and constructing the map with help of the 360 degree Lidar sensor. Different environments can be swapped within launch files to generate the map of the environment. The current most efficient algorithm used for autonomous exploration is **Rapidly Exploring Random Tree (RRT) algorithm** . The RRT Algorithm is implemented using the package from [rrt_exploration](http://wiki.ros.org/rrt_exploration) which was created to support the Kobuki robots which source files were modified and built for Turtlebot3. 

> **Original author's [Towards Data Science Story of this Project](https://mohamedfazilrobotics.medium.com/ros-autonomous-slam-using-randomly-exploring-random-tree-rrt-37186f6e3568)**

<img src="media/rrt_robot.png" alt="RRT_ROBOT" class="center" width="600"/>


### There are three key steps to run the simulations of this project.
- Step 1 : Place the robot in the Gazebo world 
- Step 2 : Perform autonomous exploration of the environment and generate the occupancy grid logs
- Step 3 : Tune logging and world configuration parameters

## Step 1 : Place the Robot in the Environment within Gazebo
Set your environment variable to the model robot to be used.
```
export TURTLEBOT3_MODEL=waffle_pi
source ~/.bashrc
```
Execute the given launch to open Gazebo with the given world file and place the robot Turtlebot3 Waffle pi model in it.
```
roslaunch ros_autonomous_slam turtlebot3_world.launch
```
Keep this process running. 

## Step 2 : Perform Autonomous exploration of the environment and generate the occupancy grid
```
roslaunch ros_autonomous_slam autonomous_explorer.launch 
```
Runs the Autonomous Explorer launch file which executes two more launch files. 

1. It starts the **SLAM** node in the Navigation stack with a custom modified RVIZ file to monitor the mapping of the environment.
2. It simultaneously starts the **autonomous explorer** which is a Python based controller to move around the robot grazing all the areas whcih helps the **SLAM** Node to complete the mapping. The default algorithm used for the exploration is RRT algorithm. 

Another explorer method which uses Bug Wall following algorithm for exploration can be tested by adding ```explorer``` argument to the launch file which takes ```[RRT,BUG_WALLFOLLOW]``` as arguments.


### Setting Exploration Region for RRT in RVIZ Window ([More Details](http://wiki.ros.org/rrt_exploration/Tutorials/singleRobot))
The RRT exploration requires a rectangular region around to be defined in the RVIZ window using four points and an starting point for exploration within the known region of the robot. The total five points must be defined in the exact sequence given below using the RVIZ **Publish Points** option. Once the 5th point is placed, the simulation begins and the Turtlebot3 will begin exploring the region.

<br>![points_sequence](media/rrt_boundary2.jpg) <br />
<br />
![RRT Mapping](media/RRT.gif)

### Exploration Progress Logging



### Manual Control 

The robot can also be manually controlled in the environment using the keyboard with the seperate launch execution given below. You can also manually explore and construct the map.
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
![Manual Gmapping](media/gmapping2.gif)

## Step 3 : Tuning Parameters

### Logging


### ROS Navigation Stack
ROS Navigation Stack requires tuning its parameters which works different for different environment types to get the Optimal SLAM and Pathplanning performance. Here is ROS's Navigation Stack parameter tuning guide for Turtlebot3.
[Turtlebot3 Navigation Parameter Tuning Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide)
<br />
![Nav](media/navigation3.gif)
![Nav](media/navigation4.gif)
### Great!!! Now we Have successfully accomplished our SLAM task with help of ROS tools. 


## Prerequisites, Dependencies, and Setup
### ROS Installation
Both Ubuntu 16 ([ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)) and Ubuntu 18 ([ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)) are compatible with this project, for MSC Lab purposes I used Ubuntu 16. Note: it's recommended to install and use [catkin_build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) as it is more flexible than ```catkin_make```.

### Gazebo ROS Installation
The main Gazebo Simulator need to be installed separately. 
[Gazebo Installation](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
Test it's installed with:
```
gazebo
```
After installing Gazebo, Gazebo-ROS interoperability is enabled with some Debian package installs:
```
# ROS Kinetic
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
Replace `kinetic` with your version of ROS anywhere Debian packages show up. 

### Turtlebot3 packages([More Details](http://wiki.ros.org/turtlebot3))
The Turtlebot3 ROS packages can be either downloaded and built from source files in your workspace
or else directly installed from the linux terminal. Either way works, I would recommend doing both as it installs all the missing dependencies required automatically.


#### Direct Installation
```
source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-turtlebot3-msgs
sudo apt-get install ros-kinetic-turtlebot3
```
Building the packages
```
cd catkin_ws/src
git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3
git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations
cd ..
catkin_make
source /devel/setup.bash
```

### [Navigation Stack](http://wiki.ros.org/navigation)
The Navigation stack can also be downloaded as source files to your catkin workspace and built.
```
sudo apt-get install ros-kinetic-navigation
cd catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-planning/navigation
cd ..
catkin_make
source /devel/setup.bash
```


