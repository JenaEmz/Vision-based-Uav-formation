#ifndef MAV_NAVIGATOR
#define MAV_NAVIGATOR

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Core>
#include "matrix/Quaternion.hpp"

#include "MavState.h"
#include "MavEnums.h"
#include "MavSensors.h"

#include "frame_ros.h"

class MavState;
class MavSensors;
class MavNavigator
{
  public:
    MavNavigator(MavState *state,MavSensors* sensors,const string& config_path);
    ~MavNavigator();

    void Takeoff(double x, double y, double height);
    void Land(double x, double y);
    void Hover();
    void MoveTo(double x,double y,double z,double yaw);
    void Hold(double x,double y,double z,double yaw);
    bool Mission(MissionPoint &Mission);

    //void UpdateBiasThread();
    void UpdateBias();//包含请求的发送

    void RequestCallback(const px4_csq::colocal_request &msg);//接受请求，计算共视角后发送自身特征和姿态d
    void ResponsCallback(const px4_csq::frame_rosPtr msg);
    void PubRespons(int id);

    ros::NodeHandle nh_;
    ros::Subscriber request_sub;
    ros::Publisher request_pub;
    ros::Subscriber self_respons_sub;
    //vector<ros::Subscriber> other_pose_subs;
    vector<ros::Publisher> other_respons_pubs;
    //vector<px4_csq::pose_with_state> other_states;
    bool Extractor_init = false;

    //TAT做不出来啦
    ros::Subscriber self_respons_left_sub;
    ros::Subscriber self_respons_right_sub;
    ros::Subscriber self_respons_info_sub;

    
  private:
    MavState *state_;
    MavSensors *sensors_;
    cv::FileStorage fsSettings_;
    CoLocalSystem *coLocal;
    CoLocalSystem *coLocal2;
};

inline double Distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
void LocalposeToSlam(const Eigen::Quaterniond &ned_q, const Eigen::Vector3d &ned_t,cv::Mat& Tcw);
void SlamToLocalpose(const cv::Mat& Tcw,Eigen::Quaterniond &ned_q, Eigen::Vector3d &ned_t);
#endif //MAV_STATE
