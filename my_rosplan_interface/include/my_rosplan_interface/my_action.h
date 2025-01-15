// Include the ROS header for using ROS functionalities.
#include <ros/ros.h>

// Include the ROSPlan Action Interface header.
#include "rosplan_action_interface/RPActionInterface.h"

// Include necessary message types for publishing and subscribing.
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <string>

// Define a namespace for the class, which is a common practice in C++ to organize code.
namespace KCL_rosplan {

// Declare the MyActionInterface class, inheriting from RPActionInterface.
class MyActionInterface : public RPActionInterface {

private:
    // Node handle for interacting with ROS.
    ros::NodeHandle nh;

    // ROS publisher for sending velocity commands.
    ros::Publisher Pub1;

    // ROS subscriber for receiving marker point data.
    ros::Subscriber MarkerCenter_subscriber;
    
    // ROS subscriber for receiving marker ID.
    ros::Subscriber MarkerNum_subscriber ;

    // Various private member variables for internal use.
    double width_camera;  
    double MarkerCenterX;
    double MarkerCenterY;  
    double MarkerID ; 
    bool flag;  
    double error; 
    double Thres;  
    double FinalLocX ; 
    double FinalLocY ;
    double LeastID ; 
    std::string LastLoc ; 
    

public:
    // Constructor declaration.
    MyActionInterface(ros::NodeHandle &nh);

    // Method to process action dispatch messages from ROSPlan.
    bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

    // Callback method for marker point data.
    void markerPointCallback(const geometry_msgs::Point::ConstPtr& msg);
    
    void markerNumCallback(const std_msgs::Int32::ConstPtr& msg)  ; 
};

} // Close the namespace KCL_rosplan
