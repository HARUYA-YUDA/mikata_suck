#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <iostream>
#include "arm.hpp"
#include <vector>

using namespace std;

std::vector<std::vector<double>>  armpose ={
    { 0.36809, 0.3619,-1.31161, 1.77482,-0.57369},//0
    { 0.36809, 0.3619,-1.31161, 1.77482,-0.57369},//1
    { 0.36809, 0.3619,-1.31161, 1.77482,-0.57369},//2
    { 0.36809, 0.3619,-1.31161, 1.77482,-0.57369},//3
    {-0.04136,-0.9142,-0.04451,-0.66724,-0.57369},//4
    {-0.04136,-0.9142,-0.04451,-0.66724,-0.57369},//5
    { 2.90684, 0.4632,-1.09066, 1.57334,-0.57369},//6
    { 2.90684, 0.4632,-1.09066, 1.57334,-0.57369},//7
    { 2.90684, 0.4632,-1.09066, 1.57334,-0.57369},//8
    {-0.04136,-0.9142,-0.04451,-0.66724,-0.57369},//9
    {-0.04136,-0.9142,-0.04451,-0.66724,-0.57369},//10
    {-0.04136,-0.9142,-0.04451,-0.66724,-0.57369},//11
};

int main(int argc, char **argv)
{
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
    while (ros::ok() && wp<12) {
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

