# harmonic_map_multirobot_exploration

This is a ROS package for multirobot exploration using harmonic maps. It is tested using ROS noetic and MATLAB R2023. The harmonic map transform is [my MATLAB implementation](https://github.com/TaxisNet/Harmonic_Map_Transformation) of 
[maxchaos' hntf2dsoftware package](https://github.com/maxchaos/hntf2d), a harmonic transformation that maps any compact, multiply connected planar domain onto the unit disk. The map merging package can be found [here](http://wiki.ros.org/multirobot_map_merge). For the simulation turtlebot3's gazebo package is used, available [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). There are plans to integrate the original or rewrite the transform in a language like python or cpp. 


# Installation
Install tb gazebo. There is a known bug for the multirobot simulations due to `tf_prefix` being deprecated, for a quick fix edit turtlebots edit the turtlebot's xarco files as shown [here](https://github.com/open-rmf/free_fleet/issues/60#issuecomment-920665293)
multirobot_map_merge.
laser filter. 
and this package. 
catking make the ws.

# Running single agent exploration

Run the following commands in diffrent terminals.

This launches the gazebo simulation
~~~
$ export TURTLEBOT3_MODEL=burger 
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch 
~~~

This launches the gmapping.
~~~
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods method:=gmapping
~~~

This launches a python script that extracts the boundary points from the Occupancy Grid Map.
~~~
$ rosrun harmonic_map_multirobot_exploration boundary_extraction.py
~~~

finally run the matlab script `single_controller.m` , located in the `src` folder. This script launches a new node, calculates the harmonic map transform and commands the robot to move.

TODO: put the modified launch files in this pkg


# Running multi agent exploration

Run the following commands in diffrent terminals.

This launches the gazebo simulation, gmapping and map merging for all robots.
~~~
$ export TURTLEBOT3_MODEL=burger 
$ roslaunch harmonic_map_multirobot_exploration multi_slam_and_map_merging.launch
~~~

This launches a python script that extracts the boundary points from the Occupancy Grid Map for all robots.
~~~
$ rosrun harmonic_map_multirobot_exploration  multi_boundary_extraction.py
~~~

finally run the matlab script `multi_controller.m` , located in the `src`  folder. This script launches a new node, calculates the harmonic map transform and commands the robot to move.



