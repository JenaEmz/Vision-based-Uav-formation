#ifndef MAV_COMMAND
#define MAV_COMMAND

#include <iostream>
#include <eigen3/Eigen/Core>
#include <thread>
#include <mutex>
#include <ros/ros.h>

#include "MavNavigator.h"
#include "MavState.h"
#include "MavSensors.h"
#include "MavEnums.h"

#include "gs_command.h"

using namespace std;

class MavCommand
{
public:
    MavCommand(MavNavigator* navigator,MavState* state,MavSensors* sensors,const string& name);
    ~MavCommand();

private:
    MavNavigator* navigator_;
    MavState* state_;
    MavSensors* sensors_;
    bool mission_quit = false;
    
    void CommandFromTopic(const px4_csq::gs_command& msg);
    void CommandFromStringThread();
    void MissionThread();
    ros::NodeHandle nh_;
    ros::Subscriber gs_cmd_sub_;
    
    thread mission_thread_;
    thread cmd_thread_;
    mutex mission_mtx;
    string name_;
    MissionTrajector mission_tarjector_;
};
int kbhit(void);
#endif //MAV_STATE
