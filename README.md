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


## Controller_node

This node listens to the topic `/base_scan` to get the result of the laser scan mounted on the robot. Using this data, the node corrects the angular velocity of the robot in order to steer it away from the walls. It then publishes a twist onto the topic `/cmd_vel` which modifies the behaviour of the robot in the simulation. This node also implements the services to increase and decrease speed.

### functions

- bool increase_speed(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) : implements the service /increase_speed.
- bool decrease_speed(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) : implements the service /decrease_speed
- float angular_vel(float front, float f_lft, float f_rgt) : using information of the surroundings provided with the 3 parameters, this function return a angular velocity which corrects the robot's course
- float min_sector(const sensor_msgs::LaserScan::ConstPtr& msg, int i, int j) : Splits the ranges array of the laser scan message and looks for the minimum value contained between the indices i and j.
- void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) : subscriber call back, publish a new twist according to measurments
- main : initialise all elements : node, node handle, publisher, subscriber, servers.

## user_interface_node

This node allows for a simpler use of the services at the user's disposal. Just by pressing one key, it is possible the change the speed of to reset the position of the robot. The node creates a client for each of the services and monitors the inputs. Using a switch on the input, it executes the suitable commmands.

- "p" (plus): increases the speed of the robot by 25%
- "m" (minus): decreases the speed of the robot by 25%
- "r" (reset): resets the robot to its original position
- "q" (quit): quits the user interface node


## flowchart of the code

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW3NpbXVsYXRpb25dIC0tPnwvYmFzZV9zY2FufCBCKGN1cnJlbnQgc3Vycm91bmRpbmdzKVxuICAgIEIgLS0-IHx0cmVhdG1lbnQgb24gcmFuZ2VzfCBDKG5ldyBhbmd1bGFyIHZlbG9jaXR5KVxuICAgIERbdXNlciBvciB1c2VyIGludGVyZmFjZV0gLS0-IHwvaW5jcmVhc2Vfc3BlZWR8IEUobmV3IHNwZWVkKVxuICAgIEQgLS0-IHwvZGVjcmVhc2Vfc3BlZWR8IEUgXG4gICAgQyAtLT4gRih1cGRhdGVkIHR3aXN0KVxuICAgIEUgLS0-IEZcbiAgICBGIC0tPiB8L2NtZF92ZWx8IEFcblxuICAgIEQgLS0-IHwvcmVzZXRfcG9zaXRpb25zfCBBXG4gICAgXG5cbiAgICIsIm1lcm1haWQiOnsidGhlbWUiOiJkYXJrIn0sInVwZGF0ZUVkaXRvciI6ZmFsc2UsImF1dG9TeW5jIjp0cnVlLCJ1cGRhdGVEaWFncmFtIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/edit#eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW3NpbXVsYXRpb25dIC0tPnwvYmFzZV9zY2FufCBCKGN1cnJlbnQgc3Vycm91bmRpbmdzKVxuICAgIEIgLS0-IHx0cmVhdG1lbnQgb24gcmFuZ2VzfCBDKG5ldyBhbmd1bGFyIHZlbG9jaXR5KVxuICAgIERbdXNlciBvciB1c2VyIGludGVyZmFjZV0gLS0-IHwvaW5jcmVhc2Vfc3BlZWR8IEUobmV3IHNwZWVkKVxuICAgIEQgLS0-IHwvZGVjcmVhc2Vfc3BlZWR8IEUgXG4gICAgQyAtLT4gRih1cGRhdGVkIHR3aXN0KVxuICAgIEUgLS0-IEZcbiAgICBGIC0tPiB8L2NtZF92ZWx8IEFcblxuICAgIEQgLS0-IHwvcmVzZXRfcG9zaXRpb25zfCBBXG4gICAgXG5cbiAgICIsIm1lcm1haWQiOiJ7XG4gIFwidGhlbWVcIjogXCJkYXJrXCJcbn0iLCJ1cGRhdGVFZGl0b3IiOmZhbHNlLCJhdXRvU3luYyI6dHJ1ZSwidXBkYXRlRGlhZ3JhbSI6ZmFsc2V9)

### Pseudo code of ranges treatment

```
    find minimum value of msg->ranges in sector front left
    find minimum value of msg->ranges in sector front
    find minimum value of msg->ranges in sector front right

    if we are not at a safe distance from the wall in one of the sectors then
        if the minimum is in front left then
            set angular velocity to 1.0*current_speed
        else if the minimum is in front right then
            set angular velocity to -1.0*current_speed
        else
            if closer to the left wall
                set angular velocity to 1.5*current_speed
            else
                set angular velocity to -1.5*current_speed
            end if
        end if
    end if


```
### Pseudo code of the input managment of the user interface
```
    while ros runs correctly do
        ask the user to press one of the following keys : r, p, m, q
        switch user input
        case r:
            wait for availability
            call reset service 
            break
        case p:
            wait for availability
            call reset service 
            break
        case m:
            wait for availability
            call reset service 
            break
        case q:
            exit program
        default:
            remind the users what keys are allowed
    end while

```

## Possible improvments
For now, the sharp turn at the beginning of the loop is difficult to drive through when increasing the speed above the initial value. To be able to handle faster speeds, I would need to publish corrections at a higher frequency. I should also investigate further the reason why I had to create a "quit" option to stop the node (ctrl+C does kill the program).
