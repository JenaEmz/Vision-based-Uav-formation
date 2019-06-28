#ifndef MAV_STATE
#define MAV_STATE

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <sys/time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>

#include "feature.h"
#include "colocal_request.h"
#include "Colocal/CoLocalSystem.h"

#include "MavNavigator.h"
#include "MavEnums.h"
#include "MavControlLoop.h"
#include "matrix/Quaternion.hpp"

using namespace std;

class MavSensors;
class MavNavigator;
class MavState
{
public:
  MavState(const string& name, MavControlLoop *controller);
  ~MavState();

  double get_pos(int axis); //x=0,z=2
  double get_yaw();
  double get_pos_sp(int axis);
  double get_yaw_sp();
  void set_yaw_sp(double yaw);
  void set_pos_sp(double x, double y, double z);
  void set_vel_sp(double vx, double vy, double vz);
  void get_Quaternion(Eigen::Quaterniond& q);
  void set_control_mode(ControlType ctr_type)
  {
    ctr_type_ = ctr_type;
  }
  bool MavEnableControl();

  int total_num = 3;
  int self_id;

  bool MavIsOk()
  {
    return MavOk;
  }
  geometry_msgs::PoseStamped local_position_;
  bool localposeIsOK = false;

  string name_;
  bool has_colocal_inited;
  mutex pos_mutex_;
  
  friend class MavNavigator;
  void SetBias(double x,double y,double z);
private:
  bool MavOk = false;
  MavControlLoop *controller_;
  mavros_msgs::State current_state_;

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_, local_pose_sub_, vel_sub_;
  
  
  mutex vel_setpoint_mutex_;
  mutex pos_setpoint_mutex_;
  ControlType ctr_type_ = NOT_CONTROL;

  Eigen::Vector3d target_pos_, target_vel_;
  double target_yaw = 0;
  Eigen::Vector3d mav_pos, mav_vel, mav_euler;
  Eigen::Vector3d bias;
  double bias_yaw = 0;
  matrix::Quatf mav_q;
  double mav_yaw;

  

  void MavStateCallback(const mavros_msgs::State::ConstPtr &msg);
  void MavPoseCallback(const geometry_msgs::PoseStamped &msg);
  void MavVelCallback(const geometry_msgs::TwistStamped &msg);

  /*
  Eigen::Vector3d Pub_PosSp();
  Eigen::Vector3d Pub_VelSp();
  Eigen::Vector3d Pub_YawSp();
  Eigen::Vector3d Pub_Pos();
  Eigen::Vector3d Pub_Vel();
  Eigen::Vector3d Pub_Yaw();
*/
};

#endif //MAV_STATE
