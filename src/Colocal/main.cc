
#include "ImageGrabber.h"

#include "features.h"

#include "CoLocalSystem.h"
#include "feature_coder.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_formation_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string config_file;
    private_nh.param("config_file_location", config_file, std::string(""));
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    string uav_name;
    private_nh.param("name", uav_name, std::string(""));
    System coLocal(fsSettings,false);

    std::string pic_path;
    private_nh.param("pic_file_location", pic_path,std::string(""));
    std::cout<<"pic path:"<<pic_path<<std::endl;
    cv::Mat left0(cv::imread(pic_path+"/left0.jpg", 0));
    cv::Mat right0(cv::imread(pic_path+"/right0.jpg", 0));

    //测试 压缩和解压特征点
    /*std::vector<uchar> bitstream;
    //coLocal.GenerateFeatureBitstream(left0,right0,bitstream);

    cv::Mat left1(cv::imread(pic_path+"/left3.jpg", 0));
    cv::Mat right1(cv::imread(pic_path+"/right3.jpg", 0));
    cv::Mat res;
    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    cv::imshow("First",left0);
    
    //coLocal.TrackStereo(left0,right0,0);
    //std::cout<< coLocal.TrackStereo(left1,right1,1);
    res = coLocal.TrackFromBitstream(bitstream,init_pose,left1,right1);
    if(!res.empty())
    {
        std::cout<<"Success get pose from bitstream,pose is "<<res<<std::endl;
    }
    else
    {
        std::cout<<"Faild get pose"<<std::endl;
    }
    
    /*std::string config_file;
    private_nh.param("config_file_location", config_file, std::string(""));
    std::string pic_path;
    private_nh.param("pic_file_location", pic_path,std::string(""));
    int use_ros_sub = 0;
    private_nh.param("use_ros_sub", use_ros_sub,0);
    std::string left_topic,right_topic,uav_name;
    private_nh.param("left_camera_topic", left_topic, std::string(""));
    private_nh.param("right_camera_topic", right_topic, std::string(""));
    private_nh.param("name", uav_name, std::string(""));

    auto frame_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 10);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cout << "Failed to open settings file at: " << config_file << endl;
        exit(-1);
    }
    CoLocalSystem coLocal(fsSettings);
    ImageGrabber igb(&coLocal);

    // Load settings related to stereo calibration
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }
    /*if (use_ros_sub)
    {
        message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
        sync.registerCallback(boost::bind(&ImageGrabber::RosStereo, &igb, _1, _2));
    }
    else
    {
        // 读第一个飞机
        cv::Mat left0(cv::imread(pic_path+"left0.jpg", 0));
        cv::Mat right0(cv::imread(pic_path+"right0.jpg", 0));
        cout << pic_path << endl;
        igb.OpencvStereo(left0,right0);
        // 读第二个飞机的图像
        cv::Mat left1(cv::imread(pic_path+"left3.jpg", 0));
        cv::Mat right1(cv::imread(pic_path+"right3.jpg", 0));
        cv::Mat RelPose = igb.OpencvStereo_Other(left1,right1);

        cv::Mat _R_matrix = cv::Mat::zeros(3, 3, CV_32F);   // rotation matrix
        cv::Mat _t_matrix = cv::Mat::zeros(3, 1, CV_32F); // translation matrix
        if(!RelPose.empty())
        {
            _R_matrix = RelPose.rowRange(0, 3).colRange(0, 3);
            _t_matrix = RelPose.rowRange(0, 3).col(3);
        }
    }*/
    /*cv::Mat left0(cv::imread(pic_path+"left0.jpg", 0));
    cv::Mat right0(cv::imread(pic_path+"right0.jpg", 0));
    igb.OpencvStereo(left0,right0);
    ros::Publisher bitstream_pub = nh.advertise<compression::msg_features>(bitstreamTopic, 1000, true);
    igb.pubFeature(left0,right0); */

    while(1)
    {
        cv::waitKey(1);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}