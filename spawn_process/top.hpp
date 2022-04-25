// #define SC_INCLUDE_DYNAMIC_PROCESSES
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
        // SC_THREAD(main);
        // publisher();
        int argc; 
        char **argv;
        ros::init(argc, argv, "publish");
        ros::start();
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
        geometry_msgs::Twist msg;
        msg.linear.x = 5;
        msg.linear.y = 0;
        msg.linear.z = 0;
        pub.publish(msg);
        ros::spin();
       
    }

// SC_MODULE(top) 
// {
	

// 	SC_CTOR(top) 

// 	{	
//         int argc; 
//         char **argv;
//         ros::init(argc, argv, "publish");
//         ros::start();
//         ros::NodeHandle n;
//         ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
//         geometry_msgs::Twist msg;
//         msg.linear.x = 5;
//         msg.linear.y = 0;
//         msg.linear.z = 0;
//         pub.publish(msg);
        
//         ros::spin();
// 	}

// };


// void main()
//     {
//         // while (1)
//         // std::cout <<"main";
//         // publisher();
        
        
//         // sc_process_handle publish = sc_spawn(sc_bind(&top::publisher, this));
//         // wait(1,SC_MS);
//     }


void publisher ()
{
    int argc; 
    char **argv;
    ros::init(argc, argv, "publish");
    ros::start();
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    // ros::Rate loop_rate(10);
    // std_msgs::String msg;
    // std::stringstream ss;
    geometry_msgs::Twist msg;
    std::cout <<"Test";
    msg.linear.x = 5;
    msg.linear.y = 0;
    msg.linear.z = 0;

    // msg.angular.z = 5;
    for (int i =0; i<3 ; i++)
    pub.publish(msg);
    ros::spin();

    // for (int i=0; i++ ; i<10)
    // {
        // msg.data = ss.str();
        // ss << "hello world " << i;
        // ROS_INFO("%s", msg.data.c_str());
    // std_msgs::Float64 msg1;
    // msg1.data = 10.0;
    // chatter_pub.publish(msg1);
        // wait(10, SC_SEC);
        // loop_rate.sleep();
    // }
    // chatter_pub.publish(msg);
    // loop_rate.sleep();
}

};