#include "MavNavigator.h"

MavNavigator::MavNavigator(MavState *state, MavSensors *sensors, const string &config_path) : state_(state), sensors_(sensors)
{
    fsSettings_ = cv::FileStorage(config_path, cv::FileStorage::READ);
    coLocal = new CoLocalSystem(fsSettings_);
    //coLocal2 = new CoLocalSystem(fsSettings_);
    //多机通信部分
    request_sub = nh_.subscribe("/request_list", 10, &MavNavigator::RequestCallback, this);
    request_pub = nh_.advertise<px4_csq::colocal_request>("/request_list", 10);
    self_respons_sub = nh_.subscribe(state->name_ + "/Respons", 10, &MavNavigator::ResponsCallback, this);

    //编队部分
    formation_target_sub = nh_.subscribe(state->name_ + "/formation_msg", 10, &MavNavigator::FormationCallback, this);
    for (int i = 0; i < state_->total_num; i++)
    {
        string bitstreamTopic = "/uav" + to_string(i) + "/Respons";
        other_respons_pubs.emplace_back(nh_.advertise<px4_csq::frame_ros>(bitstreamTopic, 1000, true));
    }
    formation_pos << 0,0,0;
}

MavNavigator::~MavNavigator()
{
}

void MavNavigator::Takeoff(double x, double y, double height = 4)
{
    state_->set_pos_sp(x, y, height);
    state_->set_control_mode(NORMAL_POSITION);
}
void MavNavigator::Land(double x, double y)
{
    state_->set_pos_sp(x, y, -1.0);
    state_->set_control_mode(NORMAL_POSITION);
}
void MavNavigator::Hover()
{
    state_->set_pos_sp(state_->get_pos(0), state_->get_pos(1), state_->get_pos(2));
    state_->set_control_mode(NORMAL_POSITION);
}
void MavNavigator::MoveTo(double x, double y, double z, double yaw)
{
    state_->set_pos_sp(x, y, z);
    state_->set_yaw_sp(yaw);
    state_->set_control_mode(NORMAL_POSITION);
}
void MavNavigator::Hold(double x, double y, double z, double yaw)
{
    state_->set_pos_sp(x, y, z);
    state_->set_yaw_sp(yaw);
    state_->set_control_mode(NORMAL_POSITION);
}
bool MavNavigator::Mission(MissionPoint &Mission)
{
    switch (Mission.type)
    {
    case TAKEOFF:
        if (Mission.start == false)
        {
            Mission.x = state_->get_pos(0);
            Mission.y = state_->get_pos(1);
            Mission.start = true;
        }
        Takeoff(Mission.x, Mission.y, 3);
        if (state_->get_pos(2) > (3 - 0.05))
            return true;
        else
            return false;
        break;
    case LAND:
        if (Mission.start == false)
        {
            Mission.x = state_->get_pos(0);
            Mission.y = state_->get_pos(1);
            Mission.start = true;
        }
        Land(Mission.x, Mission.y);
        if (state_->get_pos(2) < 0.1)
            return true;
        else
            return false;
        break;
    case MOVE_TO:
        MoveTo(Mission.x, Mission.y, Mission.z, Mission.yaw);
        if (Distance(Mission.x, Mission.y, state_->get_pos(0), state_->get_pos(1)) < 0.2 && (abs(Mission.z - state_->get_pos(2)) < 0.1))
            return true;
        else
            return false;
        break;
    case FORMATE:
        if(state_->has_colocal_inited)
        {
            MoveTo(formation_pos(0), formation_pos(1), formation_pos(2),formation_yaw);
            struct timeval cur_time;
            gettimeofday(&cur_time, NULL);
            current_colocal_time = cur_time.tv_sec*1000000 + cur_time.tv_usec;
            if((current_colocal_time - last_colocal_time)>1000000*(colocal_interval+colocal_interval_offset))
            {
                UpdateBias();
                last_colocal_time = current_colocal_time;
                std::random_device rd;
                std::default_random_engine rng {rd()};
                colocal_interval_offset = colocal_time_norm(rng);
                if(colocal_interval_offset<-1)colocal_interval_offset = -1;
                else if(colocal_interval_offset>1)colocal_interval_offset=1;
            }
        }
        else
        {
            UpdateBias();
            if(state_->has_colocal_inited)
            {
                struct timeval cur_time;
                gettimeofday(&cur_time, NULL);
                last_colocal_time = cur_time.tv_sec*1000000 + cur_time.tv_usec;
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(1));
        }

        return false;
        break;
    case HOLD:
        if (Mission.start == false)
        {
            Mission.x = state_->get_pos(0);
            Mission.y = state_->get_pos(1);
            Mission.z = state_->get_pos(2);
            Mission.start = true;
        }
        Hold(Mission.x, Mission.y, Mission.z, Mission.yaw);
        return false;
        break;
    }
}

void MavNavigator::UpdateBias()
{
    px4_csq::colocal_request msg;
    msg.has_inited = state_->has_colocal_inited;
    msg.x = state_->mav_pos(0);
    msg.y = state_->mav_pos(1);
    msg.z = state_->mav_pos(2);
    msg.id = state_->self_id;
    if (state_->self_id == 0)
        return;
    request_pub.publish(msg);

    //测试位姿估计
    /*cv::Mat left0(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
    cv::Mat left1(cv::imread("/home/jena/csq_ws/uav1-1_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/uav1-1_right.jpg", 0));
    cv::Mat res;
    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    res = coLocal->TrackFromImage(left0,right0,left1,right1);
    std::cout<<res<<std::endl;*/
    /*cv::Mat left0(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
    cv::Mat left1(cv::imread("/home/jena/csq_ws/uav1-1_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/uav1-1_right.jpg", 0));

    //测试 压缩和解压特征点
    std::vector<uchar> bitstream;
    coLocal->GenerateFeatureBitstream(left0,right0,bitstream);
    orb_formation::feature msg;
    msg.data.assign(bitstream.begin(),bitstream.end());
    std::vector<uchar> img_bitstream(msg.data.begin(), msg.data.end());

    cv::Mat res;
    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    
    //coLocal.TrackStereo(left0,right0,0);
    //std::cout<< coLocal.TrackStereo(left1,right1,1);
    res = coLocal->TrackFromBitstream(bitstream,init_pose,left1,right1);
    std::cout<<res<<std::endl;*/
}
void MavNavigator::RequestCallback(const px4_csq::colocal_request &msg)
{
    if (msg.id == state_->self_id || state_->has_colocal_inited == false)
        return;
    if ( state_->self_id == 0 )
    {
        //std::cout << "the" << state_->self_id << "get request from " << msg.id << std::endl;
        PubRespons(msg.id);
    }
    else 
    {
        //计算共视角先

        //
    }
}

void MavNavigator::PubRespons(int id)
{
    px4_csq::frame_ros msg;
    msg.nrobotid = state_->self_id;
    cv::Mat right, left;
    //std::cout << "Pub respons from:" << state_->self_id << std::endl;
    //sensors_->GetStereoImage(left, right);
    
    //cv::imwrite("/home/jena/csq_ws/wrong_0_0.jpg",left);
    //cv::imwrite("/home/jena/csq_ws/wrong_0_1.jpg",right);
   
    //cv::Mat left0(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
    //cv::Mat right0(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
    //coLocal->GenerateFeatureBitstream(1,left0,right0,bitstream);
    std::vector<uchar> bitstream;
    //coLocal->GenerateFeatureBitstream(1,left0, right0, bitstream);
    cv::Mat left0(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
    coLocal->GenerateLastFrame(left0,right0);
    std::vector<cv::KeyPoint> keyPointsLeft = coLocal->mTracker->mLastFrame.mvKeys;
    cv::Mat descriptorLeft = coLocal->mTracker->mLastFrame.mDescriptors;
    std::vector<cv::KeyPoint> keyPointsRight = coLocal->mTracker->mLastFrame.mvKeysRight;
    cv::Mat descriptorRight = coLocal->mTracker->mLastFrame.mDescriptorsRight;
    std::vector<float> mvuRight = coLocal->mTracker->mLastFrame.mvuRight;
    std::vector<float> mvDepth = coLocal->mTracker->mLastFrame.mvDepth;
    coLocal->mEncoder->encodeImageStereo(keyPointsLeft, descriptorLeft, keyPointsRight, descriptorRight, bitstream);
    //cv::imshow("0000",left);
    //cv::waitKey(1);
    //std::cout<<bitstream.size()<<" size"<<std::endl;
    msg.x = state_->mav_pos(0);
    msg.y = state_->mav_pos(1);
    msg.z = state_->mav_pos(2);
    msg.qw = state_->mav_q(0);
    msg.qx = state_->mav_q(1);
    msg.qy = state_->mav_q(2);
    msg.qz = state_->mav_q(3);
    //std::cout << state_->name_ << " current pos " << msg.x << " " << msg.y << " " << msg.z << std::endl;
    msg.data.assign(bitstream.begin(), bitstream.end());
    msg.mvuRight.assign(mvuRight.begin(), mvuRight.end());
    msg.mDepth.assign(mvDepth.begin(), mvDepth.end());
    other_respons_pubs[id].publish(msg);
}

void SE3ToCv(const Eigen::Quaterniond &t_Q, const Eigen::Vector3d &t, cv::Mat &cvMat)
{
    Matrix3d t_R = t_Q.matrix();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cvMat.at<float>(i, j) = t_R(i, j);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        cvMat.at<float>(i, 3) = t(i);
    }
}

void CvToSE3(const cv::Mat &cvMat, Eigen::Quaterniond &t_Q, Eigen::Vector3d &t)
{
    Eigen::Matrix<double, 3, 3> M;

    M << cvMat.at<float>(0, 0), cvMat.at<float>(0, 1), cvMat.at<float>(0, 2),
        cvMat.at<float>(1, 0), cvMat.at<float>(1, 1), cvMat.at<float>(1, 2),
        cvMat.at<float>(2, 0), cvMat.at<float>(2, 1), cvMat.at<float>(2, 2);
    t_Q = Eigen::Quaterniond(M);
    t << cvMat.at<float>(0, 3), cvMat.at<float>(1, 3), cvMat.at<float>(2, 3);
}

void toCvMatInverse(const cv::Mat &Tcw, cv::Mat& Twc)
{
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc * tcw;
    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
    twc.copyTo(Twc.rowRange(0, 3).col(3));
}

//NED中 D已经变成正数
void ENUtoNED(const Eigen::Quaterniond &enu_q, const Eigen::Vector3d &enu_t,
              Eigen::Quaterniond &ned_q, Eigen::Vector3d& ned_t)
{
    ned_t << enu_t(1),enu_t(0),-enu_t(2);
    ned_q =Eigen::Quaterniond(enu_q.w(),enu_q.y(),enu_q.x(),-enu_q.z());
}

void NEDtoENU(const Eigen::Quaterniond &enu_q, const Eigen::Vector3d &enu_t,
              Eigen::Quaterniond &ned_q, Eigen::Vector3d& ned_t)
{
    ned_t << enu_t(1),enu_t(0),-enu_t(2);
    ned_q = Eigen::Quaterniond(enu_q.w(),enu_q.y(),enu_q.x(),-enu_q.z());
}

void LocalposeToSlam(const Eigen::Quaterniond &ned_q, const Eigen::Vector3d &ned_t,cv::Mat& Tcw)
{
    Eigen::Quaterniond enu_q; Eigen::Vector3d enu_t;
    NEDtoENU(ned_q,ned_t,enu_q,enu_t);
    cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);
    SE3ToCv(enu_q,enu_t,Twc);
    toCvMatInverse(Twc,Tcw);
}

void SlamToLocalpose(const cv::Mat& Tcw,Eigen::Quaterniond &ned_q, Eigen::Vector3d &ned_t)
{
    /*Eigen::Quaterniond enu_q; Eigen::Vector3d enu_t;
    cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);;
    toCvMatInverse(Tcw,Twc);
    CvToSE3(Twc,enu_q,enu_t);
    ENUtoNED(enu_q,enu_t,ned_q,ned_t);*/
   
    CvToSE3(Tcw,ned_q,ned_t);
}

void MavNavigator::ResponsCallback(const px4_csq::frame_rosPtr msg)
{
    //std::cout<<img_bitstream.size()<<" size"<<std::endl;
    //Eigen::Quaterniond msg_q(msg->qw,msg->qx,msg->qy,msg->qz);
    Eigen::Quaterniond msg_q(msg->qw,msg->qx,msg->qy,msg->qz);
    Eigen::Vector3d msg_t(msg->x,msg->y,msg->z);
    cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
    
    //LocalposeToSlam(msg_q,msg_t,pose);

    //std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid << std::endl;
    cv::Mat right, left;
    sensors_->GetStereoImage(left, right);
    //cv::imshow("1111",left);
    //cv::waitKey(1);
    //cv::imwrite("/home/jena/csq_ws/wrong_1_0.jpg",left);
    //cv::imwrite("/home/jena/csq_ws/wrong_1_1.jpg",right);
    if (!Extractor_init)
    {
        cv::Mat left1(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
        cv::Mat right1(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
        std::vector<uchar> bitstream1;
        Extractor_init = true;
        coLocal->GenerateFeatureBitstream(1,left1, right1, bitstream1);
    }
    std::vector<uchar> bitstream(msg->data.begin(), msg->data.end());
    //test
    cv::Mat left1(cv::imread("/home/jena/csq_ws/uav1-1_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/uav1-1_right.jpg", 0));
    //test end
    std::vector<cv::KeyPoint> vDecKeypointsLeft;
    cv::Mat decDescriptorsLeft ;
    std::vector<cv::KeyPoint> vDecKeypointsRight;
    cv::Mat decDescriptorsRight;
    std::vector<unsigned int> vDecVisualWords;
    std::vector<float> mvuRight(msg->mvuRight.begin(), msg->mvuRight.end());
    std::vector<float> mvDepth(msg->mDepth.begin(), msg->mDepth.end());
    coLocal-> mDecoder->decodeImageStereo(bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);
    
    cv::Mat res;
    //coLocal-> mDecoder->decodeImageStereo(bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);
    //res = coLocal->TrackFromImage(left0,right0,left0,right0);
    res = coLocal->TrackFromGenerate(pose,left1,right1,vDecKeypointsLeft,decDescriptorsLeft,vDecKeypointsRight,decDescriptorsRight,mvuRight,mvDepth);
    
    //res = coLocal2->TrackFromImage(left0,right0,left1,right1,init_pose);
    if (!res.empty())
    {
        std::cout << res << std::endl;
        Eigen::Quaterniond ned_q;Eigen::Vector3d ned_t;
        SlamToLocalpose(res,ned_q,ned_t);
        std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid;
        std::cout<<" and local position is :"<<msg->x-ned_t(2)<<","<<msg->y+ned_t(0)<<","<<msg->z-ned_t(1)<<","<<std::endl;
        if(state_->has_colocal_inited == false)
        {
            
            state_->SetBias(msg->x-ned_t(2),msg->y+ned_t(0),3);
        }
        else
        {
            std::cout<<"ekf"<<std::endl;
        }
        

        /*std::cout<<msg->qw<<","<<msg->qx<<","<<msg->qy<<","<<msg->qz<<std::endl;
        std::cout<<msg->x<<","<<msg->y<<","<<msg->z<<","<<std::endl;

        Eigen::Quaterniond msg_q(msg->qw,msg->qx,msgbias(2)->qy,msg->qz);
        Eigen::Vector3d msg_t(msg->x,msg->y,msg->z);
        cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
        LocalposeToSlam(msg_q,msg_t,init_pose);*/
        //std::cout<<"pose retrans"<<init_pose<<std::endl;
    }
    else
    {
        //state_->has_colocal_inited = false;
        std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid;
        std::cout<<" but coloca1 failed"<<std::endl;
    }
}
void MavNavigator::FormationCallback(const px4_csq::colocal_request &msg)
{
    formation_pos(0) = msg.x;
    formation_pos(1) = msg.y;
    formation_pos(2) = msg.z;
    formation_yaw = msg.yaw;
}
/*
void MavNavigator::ResponsCallback(const px4_csq::frame_rosPtr msg)
{
    std::vector<uchar> img_bitstream(msg->data.begin(), msg->data.end());
    //std::cout<<img_bitstream.size()<<" size"<<std::endl;
    Eigen::Quaterniond msg_q(msg->qw,msg->qx,msg->qy,msg->qz);
    Eigen::Vector3d msg_t(msg->x,msg->y,msg->z);
    cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
    LocalposeToSlam(msg_q,msg_t,pose);
    /*orb_formation::feature msg1;
    msg1.x = state_->mav_pos(0);
    msg1.y = state_->mav_pos(1);
    msg1.z = state_->mav_pos(2);
    std::cout<<state_->name_<<" current pos "<<msg1.x<<" "<<msg1.y<<" "<<msg1.z<<std::endl;*/
/*
    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    Eigen::Quaterniond t_Q;
    state_->get_Quaternion(t_Q);

    std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid << std::endl;
    cv::Mat right, left;
    cv::Mat res;formation_pos
    sensors_->GetStereoImage(left, right);
    if (!Extractor_init)
    {
        cv::Mat left1(cv::imread("/home/jena/csq_ws/uav1-1_left.jpg", 0));
        cv::Mat right1(cv::imread("/home/jena/csq_ws/uav1-1_right.jpg", 0));
        std::vector<uchar> bitstream;
        Extractor_init = true;
        coLocal->GenerateFeatureBitstream(1,left1, right1, bitstream);
        coLocal2->GenerateFeatureBitstream(1,left1, right1, bitstream);
    }
    cv::Mat left0(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
    cv::Mat left1(cv::imread("/home/jena/csq_ws/uav1-1_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/uav1-1_right.jpg", 0));
    std::vector<uchar> bitstream;
    //coLocal->GenerateFeatureBitstream(1,left0, right0, bitstream);
    coLocal->GenerateLastFrame(left0,right0);
    std::vector<cv::KeyPoint> keyPointsLeft = coLocal->mTracker->mLastFrame.mvKeys;
    cv::Mat descriptorLeft = coLocal->mTracker->mLastFrame.mDescriptors;
    std::vector<cv::KeyPoint> keyPointsRight = coLocal->mTracker->mLastFrame.mvKeysRight;
    cv::Mat descriptorRight = coLocal->mTracker->mLastFrame.mDescriptorsRight;
    std::vector<float> mvuRight = coLocal->mTracker->mLastFrame.mvuRight;
    std::vector<float>& mvDepth = coLocal->mTracker->mLastFrame.mvDepth;
    coLocal->mEncoder->encodeImageStereo(keyPointsLeft, descriptorLeft, keyPointsRight, descriptorRight, bitstream);

    std::vector<cv::KeyPoint> vDecKeypointsLeft;
    cv::Mat decDescriptorsLeft ;
    std::vector<cv::KeyPoint> vDecKeypointsRight;
    cv::Mat decDescriptorsRight;
    std::vector<unsigned int> vDecVisualWords;
    coLocal2-> mDecoder->decodeImageStereo(bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);
    
    //coLocal-> mDecoder->decodeImageStereo(bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);
    //res = coLocal->TrackFromImage(left0,right0,left0,right0);
    res = coLocal2->TrackFromGenerate(pose,left1,right1,vDecKeypointsLeft,decDescriptorsLeft,vDecKeypointsRight,decDescriptorsRight,mvuRight,mvDepth);
    
    //res = coLocal2->TrackFromImage(left0,right0,left1,right1,init_pose);
    if (!res.empty())
    {
        std::cout << "colocal1 success:" << res << std::endl;
        Eigen::Quaterniond ned_q;Eigen::Vector3d ned_t;
        SlamToLocalpose(res,ned_q,ned_t);
        std::cout<<ned_q.w()<<","<<ned_q.x()<<","<<ned_q.y()<<","<<ned_q.z()<<std::endl;
        std::cout<<ned_t(0)<<","<<ned_t(1)<<","<<ned_t(2)<<","<<std::endl;
    }
    else
    {
        std::cout<<"coloca1 failed"<<std::endl;
        //res = coLocal->TrackFromBitstream(img_bitstream,init_pose,left,right);
    }
}*/