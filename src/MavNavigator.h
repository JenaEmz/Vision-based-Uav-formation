#ifndef MAV_NAVIGATOR
#define MAV_NAVIGATOR

#include <iostream>
#include <random>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <sys/time.h>
#include "matrix/Quaternion.hpp"

#include "MavState.h"
#include "MavEnums.h"
#include "MavSensors.h"

#include "frame_ros.h"
#include "util.hpp"
#include "DataRecorder.h"

using namespace ORB_SLAM2;
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

    void FormationCallback(const px4_csq::colocal_request &msg);

    //date record thread
    void DrawTrajThread(void);
    void RecordThread(void);

    void pos1Callback(const nav_msgs::Odometry &msg);
    void pos2Callback(const nav_msgs::Odometry &msg);
    void pos3Callback(const nav_msgs::Odometry &msg);

    ros::NodeHandle nh_;
    ros::Subscriber request_sub;
    ros::Publisher request_pub;
    ros::Subscriber self_respons_sub;
    //vector<ros::Subscriber> other_pose_subs;
    ros::Subscriber other_pos_sub[3];
    vector<ros::Publisher> other_respons_pubs;
    //vector<px4_csq::pose_with_state> other_states;
    bool Extractor_init = false;

    //TAT做不出来啦
    ros::Subscriber self_respons_left_sub;
    ros::Subscriber self_respons_right_sub;
    ros::Subscriber self_respons_info_sub;
    //编队信息
    ros::Subscriber formation_target_sub;
    double formation_yaw = 0;
    Eigen::Vector3d formation_pos;
    Eigen::Vector3d last_colocal_pos[3];

    Eigen::Vector3d last_record_pos[3];
    Eigen::Vector3d record_pos[3];
    bool has_communication[3];
    thread draw_thread_;
  private:
    MavState *state_;
    MavSensors *sensors_;
    cv::FileStorage fsSettings_;
    System *coLocal;
    System *coLocal2;
    int communication_interval = 2;
    double old_update_bias_time = 0;
    long last_colocal_time = 0;
    long current_colocal_time = 0;
    double colocal_interval = 4;
    double colocal_interval_offset = 0;
    std::normal_distribution<> colocal_time_norm{0,0.15};

    bool record_start = false;
    int trans_data_raw = 0;
    int trans_data = 0;
};

inline double Distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
void LocalposeToSlam(const Eigen::Quaterniond &ned_q, const Eigen::Vector3d &ned_t,cv::Mat& Tcw);
void LocalposeToSlam_test(const Eigen::Quaterniond &enu_q, const Eigen::Vector3d &enu_t,cv::Mat& Tcw);
void SlamToLocalpose(const cv::Mat& Tcw,Eigen::Quaterniond &ned_q, Eigen::Vector3d &ned_t);
void SlamToLocalpose_test(const cv::Mat& Tcw,Eigen::Quaterniond &enu_q, Eigen::Vector3d &enu_t);
#endif //MAV_STATE
