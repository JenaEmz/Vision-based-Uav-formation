#ifndef MAV_CONTROL_LOOP
#define MAV_CONTROL_LOOP

#include <iostream>
#include <thread>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>

#include "MavEnums.h"

using namespace std;
using namespace Eigen;

class MavControlLoop
{
public:
    MavControlLoop(const string name);
    ~MavControlLoop();
    void ControlLoopThread(void);
    void ControlLoopThread(Eigen::Vector3d& pos_sp,Eigen::Vector3d& pos,double yaw,double yaw_sp);
    void ControlLoopThread(Eigen::Vector3d& vel_sp,double yaw,double yaw_sp);
    bool arm();
    bool setOffboard();

    ros::Publisher set_vel_pub_;
private:
    string name_;
    ros::NodeHandle nh_;
    ros::Time last_offboard = ros::Time::now();
    ros::ServiceClient set_mode_client_;

    double yaw_P_ = 1.3;
    Eigen::Matrix3d pos_P_;

};
double constraind(double val, double min, double max);
void ConstrainVector(Eigen::Vector3d& target, double max_xy, double max_z);

#endif //MAV_STATE
