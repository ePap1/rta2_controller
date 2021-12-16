#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/LaserScan.h"
#include <string.h>
#include "std_srvs/Empty.h"


int main(int argc, char **argv)
{
    // Initialisation of the node and of its node handle
	ros::init(argc, argv, "user_interface");
    ros::NodeHandle nh;

	//Clients to increase speed, decrease it and reset the position
    ros::ServiceClient client_p = nh.serviceClient<std_srvs::Empty>("/increase_speed");
    ros::ServiceClient client_m = nh.serviceClient<std_srvs::Empty>("/decrease_speed");
    ros::ServiceClient client_r = nh.serviceClient<std_srvs::Empty>("/reset_positions");
    
    // service message to call the clients
    std_srvs::Empty srv;
    // character to store the key pressed by the user
    char input;

    // monitoring of the imputs as long as ros is running correctly      
    while(ros::ok()){
        //getting input
        ROS_INFO("Press r to reset the position, p to increase the speed and m for decrease it");    
        std::cin >> input;

        //We have three cases, other keys are considered as invalid
        switch (input){
        case 'r':
            client_r.waitForExistence();        
            if (client_r.call(srv)){
                ROS_INFO("Reset position");
                break;
            }
            else{
                ROS_ERROR("Failed to call service reset_positions");
                return 1;
            }

        case 'p':
            client_p.waitForExistence();
            if (client_p.call(srv)){
                ROS_INFO("Speed increased");
                break;
            }
            else{
                ROS_ERROR("Failed to call service increase_speed");
                return 1;
            }

        case 'm':
            client_m.waitForExistence();
            if (client_m.call(srv)){
                ROS_INFO("Speed decreased");
                break;
            }
            else{
                ROS_ERROR("Failed to call service decrease_speed");
                return 1;            
            }

        default:
            ROS_INFO("Please press one of admissible keys");
        }
    }    

    return 0;

  
}