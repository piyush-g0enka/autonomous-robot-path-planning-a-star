# TurtleBot Waffle A* Path Planning

This repository contains code for Path Planning using A* algorithm for a turtlebot waffle robot. We came 3rd place in a course-wide competition.
 
## YouTube Video
[![robot Video](https://github.com/piyush-g0enka/autonomous-robot-path-planning-a-star/blob/main/images/thumbnail.png)](https://www.youtube.com/watch?v=HkGXxsGaqz4)


## Usage

``` bash

To run only the algorithm (debugging):

$ python3 2d_stuff/algorithm.py

To run the algorithm on the turtlebot, place the ROS2 humble package turtlebot3_project3 in a colcon workspace and build it.
Make sure the robot is connected to the ROS2 network and can take in /cmd_vel messages. Then run the following-

$ python3 turtlebot3_project3/turtlebot3_project3/turtlebot_controller_interface.py

```
