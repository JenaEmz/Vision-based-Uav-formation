#ifndef MAV_PX4
#define MAV_PX4

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "MavState.h"
#include "MavControlLoop.h"
#include "MavSensors.h"
#include "MavNavigator.h"
#include "MavCommand.h"

using namespace std;
class MavPX4
{
    
public:
MavPX4();
    MavPX4(const string& name);
    ~MavPX4();

private:
    string name_;
    
    ros::NodeHandle nh_;
    MavControlLoop* control_loop_;
    MavState* state_;
    MavSensors* sensors_;
    MavCommand* command_handle_;
    MavNavigator* navigator_;
    string stereo_name;
};

#endif //MAV
