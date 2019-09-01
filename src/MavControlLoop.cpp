#include "MavControlLoop.h"

MavControlLoop::MavControlLoop(const string name)
{
    name_ = name;
    set_vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(name_+"/mavros/setpoint_raw/local", 10);
    pos_P_ << 0.4, 0, 0,
        0, 0.4, 0,
        0, 0, 0.5;
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(name_+"/mavros/set_mode");
}

MavControlLoop::~MavControlLoop()
{
}
void MavControlLoop::ControlLoopThread(void)
{
    mavros_msgs::PositionTarget vel_msg{};
    set_vel_pub_.publish(vel_msg);
    ++times;
    if(times>20)
    {
        setOffboard();
        times = 0;
    }
}
void MavControlLoop::ControlLoopThread(Eigen::Vector3d &pos_sp, Eigen::Vector3d &pos, double yaw, double yaw_sp)
{
    mavros_msgs::PositionTarget vel_msg{};
    double yaw_rate = 0.0f;

    vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW;
    
    Eigen::Vector3d vel_sp = pos_P_ * (pos_sp - pos);
    ConstrainVector(vel_sp, 1.5,1.5);
    vel_msg.velocity.x = vel_sp(0);
    vel_msg.velocity.y = vel_sp(1);
    vel_msg.velocity.z = vel_sp(2);
    if (abs(yaw_sp - yaw) < M_PI)
        yaw_rate = (yaw_sp - yaw) * yaw_P_;
    else
    {
        if (yaw_sp > yaw)
            yaw_rate = (yaw_sp - yaw - 2 * M_PI) * yaw_P_;
        else
            yaw_rate = (yaw_sp - yaw + 2 * M_PI) * yaw_P_;
    }
    //printf("vel:%f\n",vel_sp(2));
    //printf("yaw:%lf,%lf,%lf\n",yaw,yaw_sp,yaw_rate);
    vel_msg.yaw_rate = (yaw_rate);
    set_vel_pub_.publish(vel_msg);
    ++times;
    if(times>20)
    {
        setOffboard();
        times = 0;
    }
}
void MavControlLoop::ControlLoopThread(Eigen::Vector3d &vel_sp, double yaw, double yaw_sp)
{
    mavros_msgs::PositionTarget vel_msg{};
    double yaw_rate = 0.0f;

    vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW;
    vel_msg.velocity.x = vel_sp(0);
    vel_msg.velocity.y = vel_sp(1);
    vel_msg.velocity.z = vel_sp(2);
    if (abs(yaw_sp - yaw) < M_PI)
        yaw_rate = (yaw_sp - yaw) * yaw_P_;
    else
    {
        if (yaw_sp > yaw)
            yaw_rate = (yaw_sp - yaw - 2 * M_PI) * yaw_P_;
        else
            yaw_rate = (yaw_sp - yaw + 2 * M_PI) * yaw_P_;
    }

    vel_msg.yaw_rate = (yaw_rate);
    set_vel_pub_.publish(vel_msg);
    ++times;
    if(times>20)
    {
        setOffboard();
        times = 0;
    }
}
bool MavControlLoop::arm()
{
    auto arming_client = nh_.serviceClient<mavros_msgs::CommandBool>(name_ + "/mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;
    if (arming_client.call(srv_arm) && srv_arm.response.success)
        return true;
    else
        return false;
}
bool MavControlLoop::setOffboard()
{
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client_.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
        return true;
    else
        return false;
}

double constraind(double val, double min, double max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}

void ConstrainVector(Eigen::Vector3d &target, double max_xy, double max_z)
{
    double a = constraind(target.x(), -max_xy, max_xy);
    double b = constraind(target.y(), -max_xy, max_xy);
    double c = constraind(target.z(), -max_z, max_z);
    target << a, b, c;
}