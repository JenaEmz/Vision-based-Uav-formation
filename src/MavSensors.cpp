#include "MavSensors.h"

MavSensors::MavSensors(string name, string cam_name_1,MavState* state) : name_(name), cam_name_1_(cam_name_1)
{
    
    left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, name_ + "/stereo/left/image_raw", 1);
    right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, name_ + "/stereo/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub, *right_sub);
    sync->registerCallback(boost::bind(&MavSensors::SyncStereoCallback, this, _1, _2));
    orb_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "/mavros/vision_pose/pose", 10);
    //orb_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(name_ + "/mavros/fake_gps/mocap/tf", 10);
    fake_gps_pub_  = nh_.advertise<geometry_msgs::TransformStamped>(name_ + "/mavros/fake_gps/mocap/tf", 10);
    exvision_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "/mavros/mocap/pose", 10);
    orb_local = nullptr;
    state_ = state;
    gs_sub = nh_.subscribe("/uav1/ground_truth/state", 10, &MavSensors::gsCallback, this);
    //fake_gps_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "/mavros/gps_rtk/send_rtcm", 10);
    //nh_.subscribe(name_ + "/mavros/state", 1, &MavState::MavStateCallback, this);
    /*vo2computer_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 1, &MavSensors::SvoPoseCallback, this);

    computer2mav_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(name_ + "/mavros/vision_pose/pose", 10);

    left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, cam_name_1 + "/stereo/left/image_raw", 1);
    right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, cam_name_1 + "/stereo/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    sync = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub, *right_sub);
    sync->registerCallback(boost::bind(&MavSensors::SyncStereoCallback, this, _1, _2));
    send_vision_estimate_ = true;

    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);*/
}

MavSensors::~MavSensors()
{
    delete left_sub, right_sub, sync, tfListener_;
}

void MavSensors::SvoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
}

double MavSensors::getYaw(const geometry_msgs::Quaternion &msg)
{
    //Calculate yaw current orientation
    double roll, pitch, yaw;
    tf::Quaternion q;

    tf::quaternionMsgToTF(msg, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
}
void MavSensors::SyncStereoCallback(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1)
{
    try
    {
        imgMtx.lock();
            leftImg_ = cv_bridge::toCvShare(msg0, "mono8")->image.clone();
            rightImg_ = cv_bridge::toCvShare(msg1, "mono8")->image.clone();
            if (leftImg_.channels() == 3)
            {
                cvtColor(leftImg_, leftImg_, CV_RGB2GRAY);
                cvtColor(rightImg_, rightImg_, CV_RGB2GRAY);
                
            }
            if(orb_local!=nullptr)
            {
                if(!leftImg_.empty()&&!rightImg_.empty())
                GetPubOrbslam(leftImg_,rightImg_);
                
                else
                {
                    printf("出现空图像");
                }
                
            }
            imgMtx.unlock();
            //cv::imshow(name_,leftImg_);
            //cv::waitKey(1);
        
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());

        imgMtx.unlock();
    }
    if (record_img_once)
    {
        cv::imwrite("/home/jena/csq_ws/"+name_ +"-"+ to_string(img_id) + "_right.jpg", rightImg_);
        cv::imwrite("/home/jena/csq_ws/"+name_+"-" + to_string(img_id) + "_left.jpg", leftImg_);
        record_img_once = false;
        printf("image recorded\n");
    }
}

void MavSensors::RecordImgOnce()
{
    img_id++;
    record_img_once = true;
}

void MavSensors::GetStereoImage(cv::Mat &left, cv::Mat &Right)
{
    std::lock_guard<std::mutex> lck(imgMtx);
    left = leftImg_.clone();
    Right = rightImg_.clone();
}
void MavSensors::GetPubOrbslam(cv::Mat &left, cv::Mat &Right)
{
    ++times;
    if(times>1)
    {
    cv::Mat res;

    res = orb_local->TrackStereo(left,Right,++frame_id);
    Eigen::Quaterniond ned_q;Eigen::Vector3d ned_t;
    if(!res.empty()&&state_->has_init_q)
    {
        SlamPoseTrans::SlamToLocalpose_test(res,ned_q,ned_t);

        Eigen::Quaterniond rot_q( 0.70710678118655  ,0, 0, 0.70710678118655 );
        //Eigen::Quaterniond new_q = ned_q  ;//(ned_q*rot_q);
        //Eigen::Vector3d new_t = new_q*rot_q* ned_t  ;
        Eigen::Quaterniond new_q = ned_q;
        Eigen::Vector3d new_t = ned_t  ;
        state_->slam_pos<<new_t(0),new_t(1),new_t(2);
        //printf("origin mat:%f,%f,%f\n",res.at<float>(0, 3), res.at<float>(1, 3), res.at<float>(2, 3));
        if(abs(last_x-new_t(0))>0.5||abs(last_y-new_t(1))>0.5)
        return;

        last_x = new_t(0);
        last_y = new_t(1);
        if(times >0)
        {
        vision_pos_ENU_.header.stamp = ros::Time::now();
        
        vision_pos_ENU_.pose.position.x = new_t(0);
        vision_pos_ENU_.pose.position.y = new_t(1);
        vision_pos_ENU_.pose.position.z = new_t(2);
        //if(state_->self_id==1)
        //printf("gs:%f,%f orb:%f,%f,local %f,%f\n",gs_pos(0),gs_pos(1),new_t(0),new_t(1),state_->mav_pos(0),state_->mav_pos(1));
        vision_pos_ENU_.pose.orientation.w = new_q.w();
        vision_pos_ENU_.pose.orientation.x = new_q.x();
        vision_pos_ENU_.pose.orientation.y = new_q.y();
        vision_pos_ENU_.pose.orientation.z = new_q.z();
        /*vision_pos_ENU_.header.stamp = ros::Time::now();
        vision_pos_ENU_.transform.translation.x = new_t(0);
        vision_pos_ENU_.transform.translation.y = new_t(1);
        vision_pos_ENU_.transform.translation.z = new_t(2);*/
        //if(state_->self_id==1)
        //printf("gs:%f,%f orb:%f,%f,local %f,%f\n",gs_pos(0),gs_pos(1),new_t(0),new_t(1),state_->mav_pos(0),state_->mav_pos(1));
        /*vision_pos_ENU_.transform.rotation.w = new_q.w();
        vision_pos_ENU_.transform.rotation.x = new_q.x();
        vision_pos_ENU_.transform.rotation.y = new_q.y();
        vision_pos_ENU_.transform.rotation.z = new_q.z();*/
        orb_pub_.publish(vision_pos_ENU_);
        times=0;
        /*fake_gps_msg.header.stamp = ros::Time::now();
        fake_gps_msg.transform.translation.x = new_t(0);
        fake_gps_msg.transform.translation.y = new_t(1);
        fake_gps_msg.transform.translation.z = new_t(2);
        //if(state_->self_id==1)
        //printf("gs:%f,%f orb:%f,%f,local %f,%f\n",gs_pos(0),gs_pos(1),new_t(0),new_t(1),state_->mav_pos(0),state_->mav_pos(1));
        fake_gps_msg.transform.rotation.w = new_q.w();
        fake_gps_msg.transform.rotation.x = new_q.x();
        fake_gps_msg.transform.rotation.y = new_q.y();
        fake_gps_msg.transform.rotation.z = new_q.z();
        fake_gps_pub_.publish(fake_gps_msg);*/
        }
    }
    }
    //exvision_pub_.publish(vision_pos_ENU_);
}
void MavSensors::gsCallback(const nav_msgs::Odometry &msg)
{
    gs_pos(0) =  msg.pose.pose.position.x;
    gs_pos(1) =  msg.pose.pose.position.y;
    gs_pos(2) =  msg.pose.pose.position.z;
    gs_q = Eigen::Quaterniond(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z) ;
}
