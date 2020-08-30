# USI Angry Turtle

Robotics 2020 - Università della Svizzera Italiana

**Marco Ferri - Assignment 1**

All the tasks have been implemented.


## Demo

A working demo of the project can be find here: https://youtu.be/rWiowK1EY3U


## Get started

For executing the program, you need a working ROS enviroinment on Ubuntu with TurtleSim installed. Then, it can be executed by building the catkin package with name `usi_angry_turtle` and running the following commands inside (different tabs of) the terminal:

```
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key 
rosrun usi_angry_turtle program.py
```

Remember to kill all the processes with CTRL+C when you have finished.


## Usage

The program is built to handle a turtle called `turtle_writer` which normally writes USI on the screen and an arbitrary number of turtles called `offenders`, that randomly move inside the available space. The number of offenders can be controlled inside the `main` function of `program.py` by changing the parameter `turtles_number` (default is `10`).

Please note that `turtle1` is special, because it can also be teleoperated by using the terminal tab previously designated for it. Because of this, `turtle1` is also the only offender which respawns once it get caught and killed. That is why the program never actually terminates and should be stopped manually at the end of the test.

Last but not least, it should be considered that while the `writer` chases an `offender`, it aims not directly for the offender's pose but instead tries to look ahead of some meters (as defined in the method `UsiAngryTurtle.calculate_pose_ahead()`). This causes the `writer` to get crazy sometimes, when some particular poses occurs for the `offender` it is chasing. However, this behaviour can be toggled off by acting on the proper parameter `look_ahead` inside the `main` function.



