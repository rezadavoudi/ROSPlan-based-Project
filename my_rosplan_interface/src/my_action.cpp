#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <string>
#include <iostream>
#include <cstdlib>





namespace KCL_rosplan {
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	
		Pub1 = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		
		// Initialize a subscriber to the "/marker_point" topic.
        	// The 'markerPointCallback' function of this class will handle the incoming messages.
        	MarkerCenter_subscriber = nh.subscribe("/marker/center_loc", 10, &MyActionInterface::markerPointCallback, this);
        	MarkerNum_subscriber = nh.subscribe("/marker/id_number", 10, &MyActionInterface::markerNumCallback, this);
        	
        	// Initializing all the necessary variables.
        	MarkerCenterX = 0.0; 
        	MarkerCenterY = 0.0;      
        	width_camera = 320.0;        // Set the camera's width, used for calculating the error.
        	flag = true;                 // A flag used to control the main loop in the action.
        	error = 0.0;                 // Initialize the error to zero.
        	Thres = 18.0;                // Set the threshold for the pixel error.
        	MarkerID = 0.0 ; 
        	FinalLocX = 0 ; 
        	FinalLocY = 0 ;
        	
        	LeastID = 200.0;
        	LastLoc = "loc4";
        	
	
	
	}
	
	
	
	// Callback function for the '/marker_point' topic subscriber.
    // It receives the position of a marer, particularly focusing on the x-coordinate.
    void MyActionInterface::markerPointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    
        // Update the MarkerCenterX and MarkerCenterY variables with the x-coordinate and y-coordinate from the message.
        MarkerCenterX = msg->x;
        MarkerCenterY = msg->y;
        

        


    }
    
    void MyActionInterface::markerNumCallback(const std_msgs::Int32::ConstPtr& msg) {
    
        // Update the MarkerCenterX and MarkerCenterY variables with the x-coordinate and y-coordinate from the message.
        MarkerID= msg->data;
 


    }
    
    
    
    
    
    
	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
{
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	ac.waitForServer();
		
	if (msg -> name == "move_to") {
		// here the implementation of the action
		std::cout << "The Robot is Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		
		
		goal.target_pose.header.frame_id = "map" ;
		goal.target_pose.pose.orientation.w = 1.0;
		
		if(msg->parameters[2].value == "loc1"){
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			
		}
		else if (msg->parameters[2].value == "loc2"){
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			
		}
		else if (msg->parameters[2].value == "loc3"){
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			
		}
		else if (msg->parameters[2].value == "loc4"){
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 2.75;
			
		}
		else if (msg->parameters[2].value == "loc0"){
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = 1.5;
			
		}
		ac.sendGoal(goal);
		ac.waitForResult();
	}
	
	
	else if (msg -> name == "identify") {
	
		std::cout << "The Robot is roatating to find the marker in location" << std::endl;
		
		while (flag) {
		
			error = std::abs(MarkerCenterX - width_camera);
			
			geometry_msgs::Twist twist;
                	twist.angular.z = 0.5;
                	
                	Pub1.publish(twist);
                	
                	if (error < Thres) {
                	
                		flag = false ; 
                		
                		std::cout << "GOOOOOOOOOD , The marker found , id of marker is: x = " << MarkerID << std::endl;
                		
                		if(MarkerID < LeastID){
                		
                			LeastID = MarkerID ; 
                			
                			LastLoc = msg->parameters[1].value ;
                			
                			nh.setParam("last_loc", LastLoc);
                			nh.setParam("least_id", LeastID);

					// Log the updated goal location
					std::cout << "the Least marker id is updated : " << LeastID 
						  << " at goal location x = " << LastLoc << std::endl;
                		}
                		
                		else {
                		
                			// Log the updated goal location
					std::cout << "the Least marker id is updated : " << LeastID 
						  << " at goal location x = " << LastLoc << std::endl;
                		}
                		
                		twist.angular.z = 0.0;
                		
                		Pub1.publish(twist);
                	}
		
		}
		
		flag = true ; 
		
		 
	
	}
	
	else if (msg -> name == "verify_targets") {
	
		nh.getParam("last_loc", LastLoc);
		nh.getParam("least_id", LeastID);
		
		std::cout << "The Robot is Going back to " << LastLoc << " with this ID " << LeastID << std::endl;
		
		goal.target_pose.header.frame_id = "map" ;
		goal.target_pose.pose.orientation.w = 1.0;
		
		if(LastLoc == "loc1"){
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			
		}
		else if (LastLoc == "loc2"){
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			
		}
		else if (LastLoc == "loc3"){
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			
		}
		else if (LastLoc == "loc4"){
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 2.75;
			
		}
		else if (LastLoc == "loc0"){
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = 1.5;
			
		}
		ac.sendGoal(goal);
		ac.waitForResult();
	
	
	}
	
	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "assignment_plan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
