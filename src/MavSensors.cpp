#include "MavSensors.h"

MavSensors::MavSensors(string name, string cam_name_1) : name_(name), cam_name_1_(cam_name_1)
{
    left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, name_ + "/stereo/left/image_raw", 1);
    right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, name_ + "/stereo/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub, *right_sub);
    sync->registerCallback(boost::bind(&MavSensors::SyncStereoCallback, this, _1, _2));
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
    /*svo_position_ = *msg;
    if(!state_->localposeIsOK)return;
    if (ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0))
    {
        // svo_position is the first pose message after initialization/recovery, need to set svo_init_pos
        ROS_INFO("svo_init_pos = local_position");
        svo_init_pos_ = state_->local_position_;

        // Transformation from world to svo_init
        transformStamped_.header.stamp = svo_position_.header.stamp;
        transformStamped_.header.frame_id = "world";
        transformStamped_.child_frame_id = "svo_init";
        transformStamped_.transform.translation.x = svo_init_pos_.pose.position.x;
        transformStamped_.transform.translation.y = svo_init_pos_.pose.position.y;
        transformStamped_.transform.translation.z = svo_init_pos_.pose.position.z;
        transformStamped_.transform.rotation = svo_init_pos_.pose.orientation;

        svoinitToWorld.sendTransform(transformStamped_);
    }

    // Transformation from svo_init to drone_vision
    last_svo_estimate_ = ros::Time::now();
    transformStamped_.header.stamp = svo_position_.header.stamp;
    transformStamped_.header.frame_id = "svo_init";
    transformStamped_.child_frame_id = "drone_vision";
    transformStamped_.transform.translation.x = svo_position_.pose.pose.position.x;
    transformStamped_.transform.translation.y = svo_position_.pose.pose.position.y;
    transformStamped_.transform.translation.z = svo_position_.pose.pose.position.z;
    transformStamped_.transform.rotation = svo_position_.pose.pose.orientation;
    svoToSvoinit.sendTransform(transformStamped_);
    if (send_vision_estimate_)
    {
        try
        {
            // Send vision position estimate to mavros
            transformStamped_ = tfBuffer_.lookupTransform("world", "drone_vision", ros::Time(0));
            vision_pos_ENU_.header.stamp = svo_position_.header.stamp;
            vision_pos_ENU_.header.frame_id = "world";
            vision_pos_ENU_.pose.position.x = transformStamped_.transform.translation.x;
            vision_pos_ENU_.pose.position.y = transformStamped_.transform.translation.y;
            vision_pos_ENU_.pose.position.z = transformStamped_.transform.translation.z;
            vision_pos_ENU_.pose.orientation = transformStamped_.transform.rotation;
            //computer2mav_pub_.publish(vision_pos_ENU_);

            cnt++;
            if (cnt % 1 == 0)
            {
                printf("Vision position lookup: E: %f, N: %f, U: %f, yaw: %f\n", transformStamped_.transform.translation.x,
                         transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, getYaw(transformStamped_.transform.rotation));
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }*/
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
        if (imgMtx.try_lock())
        {
            leftImg_ = cv_bridge::toCvShare(msg0, "mono8")->image.clone();
            rightImg_ = cv_bridge::toCvShare(msg1, "mono8")->image.clone();
            if (leftImg_.channels() == 3)
            {
                cvtColor(leftImg_, leftImg_, CV_RGB2GRAY);
                cvtColor(rightImg_, rightImg_, CV_RGB2GRAY);
            }
            imgMtx.unlock();
            cv::imshow(name_,leftImg_);
            cv::waitKey(1);
        }
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
    leftImg_.copyTo(left);
    rightImg_.copyTo(Right);
}