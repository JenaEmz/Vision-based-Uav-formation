#include <ros/ros.h>
#include "MavPX4.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"px4_csq");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    int id;
    private_nh.param("name",id,0);
    std::cout<<"id:"<<id<<endl;
    MavPX4 mav_1 = MavPX4(to_string(id));
    ros::Rate* rate_ = new ros::Rate(20.0);
    
    while (ros::ok())
    {
        ros::spinOnce();
        rate_->sleep();
    }
    delete rate_;
    /*ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();*/
    ros::waitForShutdown();
    return 0;
}