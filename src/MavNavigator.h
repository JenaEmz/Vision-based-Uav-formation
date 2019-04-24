#ifndef MAV_NAVIGATOR
#define MAV_NAVIGATOR

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Core>

#include "MavState.h"
#include "MavEnums.h"
#include "MavSensors.h"
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
    void ResponsCallback(const orb_formation::feature::ConstPtr msg);
    void PubRespons(int id);

    ros::NodeHandle nh_;
    ros::Subscriber request_sub;
    ros::Publisher request_pub;
    ros::Subscriber self_respons_sub;
    //vector<ros::Subscriber> other_pose_subs;
    vector<ros::Publisher> other_respons_pubs;
    //vector<px4_csq::pose_with_state> other_states;
    bool Extractor_init = false;
 
  private:
    MavState *state_;
    MavSensors *sensors_;
    cv::FileStorage fsSettings_;
    CoLocalSystem *coLocal;
    
};

inline double Distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

#endif //MAV_STATE
