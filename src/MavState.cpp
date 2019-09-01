#include "MavState.h"

MavState::MavState(const string& name, MavControlLoop *controller) : controller_(controller)
{
    //自身控制部分
    name_ = name;
    MavOk = true;
    state_sub_ = nh_.subscribe(name_ + "/mavros/state", 1, &MavState::MavStateCallback, this);
    local_pose_sub_ = nh_.subscribe(name_ + "/mavros/local_position/pose", 5, &MavState::MavPoseCallback, this);
    vel_sub_ = nh_.subscribe(name_ + "/mavros/local_position/velocity", 1, &MavState::MavVelCallback, this);
 
    self_id = atoi(name_.substr(4,1).c_str());
    has_colocal_inited = false;
    if(self_id==0)has_colocal_inited = true;
    bias<<0,0,0;
}

MavState::~MavState()
{
}
void MavState::MavStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_ = *msg;
}

void MavState::SetBias(double x,double y,double z)
{
    //bias(0)+= x - mav_pos(0);
    //bias(1)+= y - mav_pos(1);
    bias(0) = x;
    bias(1) = y;
    has_colocal_inited = true;
}

void MavState::MavPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    if (!MavIsOk())
    {
        return;
    }
    if(!has_init_q)
    {
        has_init_q = true;
        init_q = Eigen::Quaterniond(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z);
        printf("init_q:%f,%f,%f,%f\n",msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z);
    }
    local_position_ = msg;
    localposeIsOK = true;
    /*mav_pos(0) = msg.pose.position.x;
    mav_pos(1) = msg.pose.position.y;
    mav_pos(2) = msg.pose.position.z;*/
    //加上偏差值
    /*mav_pos(0) = msg.pose.position.x + bias(0);
    mav_pos(1) = msg.pose.position.y + bias(1);
    mav_pos(2) = msg.pose.position.z + bias(2);*/
    mav_pos(0) = slam_pos(0) + bias(0);
    mav_pos(1) = slam_pos(1) + bias(1);
    /*mav_pos(0) = msg.pose.position.x + bias(0);
    mav_pos(1) = msg.pose.position.y + bias(1);*/
    mav_pos(2) = msg.pose.position.z + bias(2);
    
//printf("new_t:%f,%f true %f,%f\n",slam_pos(0),slam_pos(1),msg.pose.position.x,msg.pose.position.y);
    mav_q = matrix::Quatf(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z);
    mav_yaw = matrix::Eulerf(mav_q).psi();
    switch (ctr_type_)
    {
    case NOT_CONTROL:
        break;
    case NORMAL_POSITION:
        pos_setpoint_mutex_.lock();
        controller_->ControlLoopThread(target_pos_, mav_pos, mav_yaw, target_yaw);
        pos_setpoint_mutex_.unlock();
        break;
    case NORMAL_VELOCITY:
        vel_setpoint_mutex_.lock();
        controller_->ControlLoopThread(target_vel_, mav_yaw, target_yaw);
        vel_setpoint_mutex_.unlock();
    default:
        printf("not a legal control type!\n");  
        break;
    }
}
void MavState::MavVelCallback(const geometry_msgs::TwistStamped &msg)
{
    mav_vel(0) = msg.twist.linear.x;
    mav_vel(1) = msg.twist.linear.y;
    mav_vel(2) = msg.twist.linear.z;
}
void MavState::GroundTruthCallback(const geometry_msgs::PoseStamped &msg)
{
    groundtruth(0) = msg.pose.position.x;
    groundtruth(1) = msg.pose.position.y;
    groundtruth(2) = msg.pose.position.z;
}
double MavState::get_groundtruth(int axis)
{
    return groundtruth(axis);
}
double MavState::get_pos(int axis)
{
    return mav_pos(axis);
}
double MavState::get_yaw()
{
    return mav_euler(2);
}
void MavState::get_Quaternion(Eigen::Quaterniond& q)
{
    q = Eigen::Quaterniond(mav_q(0),mav_q(1),mav_q(2),mav_q(3));
}
double MavState::get_pos_sp(int axis)
{
    return target_pos_(axis);
}
double MavState::get_yaw_sp()
{
    return target_yaw;
}
void MavState::set_yaw_sp(double yaw_sp)
{
    if(yaw_sp>M_PI||yaw_sp<-M_PI)
    {
        while(yaw_sp>M_PI)
        {
            yaw_sp -= 2 * M_PI;
        }
        while(yaw_sp<-M_PI)
        {
            yaw_sp += 2 * M_PI;
        }
    }
    target_yaw = yaw_sp;
}
void MavState::set_pos_sp(double x, double y, double z)
{
    pos_setpoint_mutex_.lock();
    target_pos_ << x, y, z;
    pos_setpoint_mutex_.unlock();
}
void MavState::set_vel_sp(double vx, double vy, double vz)
{
    vel_setpoint_mutex_.lock();
    target_vel_ << vx, vy, vz;
    vel_setpoint_mutex_.unlock();
}

bool MavState::MavEnableControl()
{
    if (!controller_->arm())
    {
        cout << name_ << " try to arm failed" << endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
        return false;
    }
    cout << name_ << " armed success" << endl;
    
    int i = 0;

    mavros_msgs::PositionTarget vel_msg{};
    vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    vel_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW;
    vel_msg.velocity.x = 0;
    vel_msg.velocity.y = 0;
    vel_msg.velocity.z = 0;
    vel_msg.header.stamp = ros::Time::now();
    //controller_->set_vel_pub_.publish(vel_msg);

    if (!controller_->setOffboard())
    {
        cout << name_ << " try to offboard failed" << endl;
            return false;
    }
    cout << name_ << " offboard success" << endl;
    return true;
}