#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/LaserScan.h"
#include <string.h>
#include "std_srvs/Empty.h"

// Distance to the walls under which a course correction is needed
float safe_dist = 1.5;

float current_speed = 1.0; 
ros::Publisher pub;


//service to add speed
bool increase_speed(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
   
    //augmentation of 25%
    current_speed = current_speed*1.25;
    return true;        
}


//service to reduce speed
bool decrease_speed(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){
    
    //reduction of 25%
    current_speed = current_speed*0.75;
    return true;
    
}


// function to avoid the walls by changing the angular velocity
float angular_vel(float front, float f_lft, float f_rgt){
    float ang_vel =0;

    // If are no longer at a safe distance from the walls
    if (f_lft < safe_dist or front< safe_dist or f_rgt < safe_dist){
        if (f_lft < front and f_lft < f_rgt)
            ang_vel = -1.0;
        else if (f_rgt < front and f_rgt < f_lft)
            ang_vel = 1.0;
        else{
            // In this case, we are facing the obstacle, we need a sharper turn
            if (f_lft < f_rgt)
                ang_vel = -2.0;
            else
                ang_vel = 2.0;
        }
    }
    return(ang_vel);
}

float min_sector(const sensor_msgs::LaserScan::ConstPtr& msg, int i, int j){
    float min = 100;
    float idx_min = i;

    for (int e=i; e<j; e++){
        if (msg->ranges[e]<min){
            min = msg->ranges[e];
            idx_min = e;
        }
    }
    return min;
}

void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Twist my_vel;
    
    //I am splitting the result of the base scan into 5 regions but 
    // I only consider the three forward ones
    float f_lft, front, f_rgt;    

    //Retrieval of the smallest value in the three forward regions
    // ranges is an array of 720 values
    
    f_lft = min_sector(msg, 144, 287);
    front = min_sector(msg, 288, 431);
    f_rgt = min_sector(msg, 432, 585);

    // According to the distance at which the obstacle is and the sector it is in
    // I adjust the angular velocity to straighten the course of the robot				
	my_vel.linear.x = current_speed;
    my_vel.angular.z = angular_vel(front, f_lft, f_rgt);
	pub.publish(my_vel);
}


int main (int argc, char **argv){
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "controller");  
	ros::NodeHandle nh;
		
	
	// Publisher for the linear and angular velocity
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);  
	// Subscriber to the laser scan
    ros::Subscriber sub = nh.subscribe("/base_scan", 1,ScanCallBack);  

    // Servers for the speed services
    ros::ServiceServer service_increase = nh.advertiseService("/increase_speed", increase_speed);
    ros::ServiceServer service_decrease = nh.advertiseService("/decrease_speed", decrease_speed);
      
    ROS_INFO("CONTROLLER ON");	
	 
	ros::spin();
	return 0;
}

