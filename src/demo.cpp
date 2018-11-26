#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <iostream>
#include "arm.hpp"
#include <vector>

using namespace std;

std::vector<std::vector<double>>  armpose ={
    //{0,-1.81724,1.409702,-1.23953,0},//01 rest
    {0,-1.10759,0.329867,0.997107,0},//02 prep
    //{0,0.208567,0.019900,1.368338,0},//03 pick
    {0,0.170344,-0.329870,1.734857,0},//03 pick
    {0,-1.10759,0.329867,0.997107,0},//04 prep
    {1.570796,0.523599,-1.18682,0.628319,0},//05 place
    {0,-1.10759,0.329867,0.997107,0},//06 prep
    //{0,-1.81724,1.409702,-1.23953,0},//07 rest
};

int main(int argc, char **argv)
{
    const int WP_FULL = 5;

    ROS_INFO("-----------------------");

    //magic, don't think about here.
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    Arm arm(&nh);

    //initializing way point.
    // ROS cooridnata: forward is X axis,  left direction is y axis, and up is z axis.
    int wp = 0; // Next waypoint
    sleep(1);

    //main while
    while (ros::ok() && wp<WP_FULL) {
        //ros::spinOnce();
        //way point running
        ROS_INFO("Go to WP %d",wp);
        arm.armPos(armpose[wp]);
        ROS_INFO("---");

        while(ros::ok() && arm.moveCheck()){
            ros::spinOnce();
            arm.cycle();    
        }
        ROS_INFO("---");
        wp++;
    }



    ROS_INFO("Mission complete!");
    sleep(3); // [s]
    return 0;
}

