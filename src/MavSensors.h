#ifndef MAV_SENSORS
#define MAV_SENSORS

#include <iostream>
#include <string>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "MavState.h"
class MavState;
using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    
class MavSensors
{
public:
    MavSensors(string name,string cam_name_1);
    ~MavSensors();
    
    void RecordImgOnce();
    double getYaw(const geometry_msgs::Quaternion &msg);
    void GetStereoImage(cv::Mat& left,cv::Mat& Right);
    

private:
    string name_;
    string cam_name_1_;
    bool record_img_once = false;
    int img_id = 0;
    MavState *state_;
    cv::Mat leftImg_, rightImg_;
    std::mutex imgMtx;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;
    ros::Subscriber vo2computer_sub_;
    ros::Subscriber stereo_img_sub_;
    ros::Publisher computer2mav_pub_;
    geometry_msgs::PoseWithCovarianceStamped svo_position_;
    ros::Time last_svo_estimate_;
    geometry_msgs::PoseStamped svo_init_pos_;
    tf2_ros::StaticTransformBroadcaster svoinitToWorld;
    tf2_ros::TransformBroadcaster svoToSvoinit;
    geometry_msgs::TransformStamped transformStamped_;

    int cnt;
    bool send_vision_estimate_ = false;
 
    geometry_msgs::PoseStamped vision_pos_ENU_;
    
    void SvoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    
    message_filters::Subscriber<sensor_msgs::Image>* left_sub;
    message_filters::Subscriber<sensor_msgs::Image>* right_sub;
    
    message_filters::Synchronizer<sync_pol>* sync;
    void SyncStereoCallback(const sensor_msgs::ImageConstPtr& msg0,const sensor_msgs::ImageConstPtr& msg1);


};

#endif //MavSensors
