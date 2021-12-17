# rta2_controller
This package is an answer to a simulated robot control problem given as second assignement for the Research Track 1 class, 2021-2022 at UniGe. This simulation is running with ROS.

The environment is a loop circuit in which a mobile robot drives. The robot must stay clear of the walls, and to do so, it is equiped with a laser scanner. To control the robot, the twist can be modify by a controller node. The user can also use services to increase or decrease the speed of the robot as well as reset its position.

## How to run 
First, compile the package in your ros workspace using
```Shell
catkin_make
```
You should then run the different nodes using
```Shell
roscore
rosrun stage_ros stageros $(rospack find rta2_controller)/world/my_world.world
rosrun rta2_controller controller_node
rosrun rta2_controller user_interface_node
```

Once the system is running, the user can interact with the simulation using the following services of type std_srvs/Empty:
- /increase_speed : increases the speed of the robot by 25%
- /decrease_speed : decreases the speed of the robot by 25%
- /reset_positions : deplaces the robot back to its original position

Services can be accessed using :
```Shell
rosservice call <nameOfService>
```
The user interface also allows the same functionalities with quicker commands.

## Nodes

### Controller_node

This node listens to the topic `/base_scan` to get the result of the laser scan mounted on the robot. Using this data, the node corrects the angular velocity of the robot in order to steer it away from the walls. It then publishes a twist onto the topic `/cmd_vel` which modifies the behaviour of the robot in the simulation. This node also implements the services to increase and decrease speed.

### functions

- 



### user_interface_node

This node allows for a simpler use of the services at the user's disposal. Just by pressing one key, it is possible the change the speed of to reset the position of the robot

- "p" (plus): increases the speed of the robot by 25%
- "m" (minus): decreases the speed of the robot by 25%
- "r" (reset): resets the robot to its original position
- "q" (quit): quits the user interface node


## flowchart of the code