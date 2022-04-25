#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"

using namespace sc_core;
using namespace std;

class top : public sc_module
{

    public:

    SC_HAS_PROCESS (top);

    top(sc_module_name name) : sc_module(name)
    {

        // int argc; 
        // char **argv;
        // ros::init(argc, argv, "publish");
        // ros::start();
        // ros::NodeHandle n;
        // ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
        // geometry_msgs::Twist msg;
        // msg.linear.x = 5;
        // msg.linear.y = 0;
        // msg.linear.z = 0;
        // pub.publish(msg);
        // ros::spin();
        SC_THREAD (main);
       
    }



void main()
    {

        
        sc_process_handle publish = sc_spawn(sc_bind(&top::publisher, this));
    
    }


void publisher ()
{
    int argc; 
    char **argv;
    ros::init(argc, argv, "publish");
    ros::start();
    ros::Rate rate(1);
    
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    geometry_msgs::Twist msg;
    std::cout <<"Test";
    msg.linear.x = 5;
    msg.linear.y = 0;
    msg.linear.z = 0;



    for (int i = 0; i < 5; i++)
    
    {
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }



}

};