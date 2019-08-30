#include "MavNavigator.h"

MavNavigator::MavNavigator(MavState *state, MavSensors *sensors, const string &config_path) : state_(state), sensors_(sensors)
{
    
    fsSettings_ = cv::FileStorage(config_path, cv::FileStorage::READ);
    if(state_->self_id == 0)
    {
        coLocal = new System(fsSettings_,true);
    }
    else
    {
        coLocal = new System(fsSettings_,false);
    }
    sensors->orb_local = coLocal;
    //coLocal2 = new CoLocalSystem(fsSettings_);
    //多机通信部分
    request_sub = nh_.subscribe("/request_list", 10, &MavNavigator::RequestCallback, this);
    request_pub = nh_.advertise<px4_csq::colocal_request>("/request_list", 10);
    self_respons_sub = nh_.subscribe(state->name_ + "/Respons", 10, &MavNavigator::ResponsCallback, this);
    
    //编队部分
    formation_target_sub = nh_.subscribe(state->name_ + "/formation_msg", 10, &MavNavigator::FormationCallback, this);
    
    other_pos_sub[0] = nh_.subscribe("/uav0/ground_truth/state", 10, &MavNavigator::pos1Callback, this);
    other_pos_sub[1] = nh_.subscribe("/uav1/ground_truth/state", 10, &MavNavigator::pos2Callback, this);
    other_pos_sub[2] = nh_.subscribe("/uav2/ground_truth/state", 10, &MavNavigator::pos3Callback, this);
    
    for (int i = 0; i < state_->total_num; i++)
    {
        string bitstreamTopic = "/uav" + to_string(i) + "/Respons";
        other_respons_pubs.emplace_back(nh_.advertise<px4_csq::frame_ros>(bitstreamTopic, 1000, true));
    }
    formation_pos << 0,0,0;

    char timeinfo[256] ={0};
    char timeinfo2[256] ={0};
    time_t nowtime = time(NULL);
    struct tm *p;
    p = gmtime(&nowtime);

    sprintf(timeinfo,"/home/jena/csq_ws/logs/%d_%d_%d_%d_%02d_/",1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min);
    sprintf(timeinfo2,"%d_%02d_",8+p->tm_hour,p->tm_min);
    string dir_name = string(timeinfo);
    mkdir(dir_name.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    string file_name = dir_name+string(timeinfo2)+"uav"+to_string(state_->self_id)+"_.txt";
    char head[] = "timestamp\tinit_state\tx\ty\tz\ttrans_data\ttrans_data_raw\tgroundtruth_x\tgroundtruth_y\tgroundtruth_z\n";

    recorder = new DataRecorder(head,file_name.c_str());
    draw_thread_ = std::thread(&MavNavigator::DrawTrajThread,this);
    record_thread_ = std::thread(&MavNavigator::RecordThread,this);
    if(state_->self_id == 0)
    {
        string gs_name = dir_name+ string(timeinfo2)+"gs.txt";
        char head[] = "timestamp\tg0x\tg0y\tg0z\tg1x\tg1y\tg1z\tg2x\tg2y\tg2z\n";
        gs_recorder =  new DataRecorder(head,gs_name.c_str());
        record_gs_thread_ = std::thread(&MavNavigator::RecordGsThread,this);
    }
    last_comm_pos<<0,0,0;
    printf("nav init\n");
}

MavNavigator::~MavNavigator()
{
}

void MavNavigator::Takeoff(double x, double y, double height = 2)
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
        Takeoff(Mission.x, Mission.y, 2);
        if (state_->get_pos(2) > (2 - 0.05))
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

        record_start = true;
        if(state_->has_colocal_inited)
        {
            MoveTo(formation_pos(0), formation_pos(1), formation_pos(2),formation_yaw);
            
            float dist_x = last_comm_pos(0)-state_->slam_pos(0);
            float dist_y = last_comm_pos(1)-state_->slam_pos(1);
            if((dist_x*dist_x+dist_y*dist_y)>1)
            {
                need_new_colocal = false;
            }

            struct timeval cur_time;
            gettimeofday(&cur_time, NULL);
            current_colocal_time = cur_time.tv_sec*1000000 + cur_time.tv_usec;
            if((current_colocal_time - last_colocal_time)>1000000*(colocal_interval)||need_new_colocal)
            {
                need_new_colocal = true;
            }
            if(need_new_colocal)
            {
                need_new_colocal = false;
                //UpdateBias();
                last_colocal_time = current_colocal_time;
                std::random_device rd;
                std::default_random_engine rng {rd()};
                colocal_interval_offset = colocal_time_norm(rng);
                if(colocal_interval_offset<-1)colocal_interval_offset = -1;
                else if(colocal_interval_offset>1)colocal_interval_offset=1;

                last_comm_pos(0) = state_->slam_pos(0);
                last_comm_pos(1) = state_->slam_pos(1);
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

            std::this_thread::sleep_for(std::chrono::duration<double>(5));
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
    /*if(state_->self_id == 1)
    {
        state_->SetBias(-1, -2, 3);
    }
    if(state_->self_id == 2)
    {
        state_->SetBias(-2, 3, 3);
    }*/
    
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
    if(msg.has_inited==true)
    {
        int chuan =rand()%5;
        if(chuan>3)return;
    }
    if ( state_->self_id == 0 ||state_->self_id == 0)
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
    sensors_->GetStereoImage(left, right);
    //cv::imwrite("/home/jena/csq_ws/wrong_0_0.jpg",left);
    //cv::imwrite("/home/jena/csq_ws/wrong_0_1.jpg",right);
    
    //coLocal->GenerateFeatureBitstream(1,left0,right0,bitstream);
    std::vector<uchar> bitstream;
    //coLocal->GenerateFeatureBitstream(1,left0, right0, bitstream);
    //test
    /*cv::Mat left0(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));*/
    //test end
    //coLocal->GenerateLastFrame(left,right);
    /*std::vector<cv::KeyPoint> keyPointsLeft = coLocal->mTracker->mLastFrame.mvKeys;
    cv::Mat descriptorLeft = coLocal->mTracker->mCurrentFrame.mDescriptors;
    std::vector<cv::KeyPoint> keyPointsRight = coLocal->mTracker->mLastFrame.mvKeysRight;
    cv::Mat descriptorRight = coLocal->mTracker->mLastFrame.mDescriptorsRight;*/
    std::vector<float> mvuRight; 
    std::vector<float> mvDepth; 
    int raw_size =0;
    cv::Mat Tcw;
    /*if(!left.empty()&&!right.empty())
    coLocal->GenerateBitstream(left,right, bitstream, mvuRight,mvDepth,raw_size );
    
    else
    {
        printf("图像为空？\n");
        return;
    }*/
    coLocal->GenerateBitstream(left,right,bitstream, mvuRight,mvDepth,raw_size );
    //coLocal->GenerateBitstream(Tcw,bitstream, mvuRight,mvDepth,raw_size );
    Eigen::Vector3d new_t;
    Eigen::Quaterniond new_q;
    /*if(!Tcw.empty())
    {
        Eigen::Quaterniond ned_q;Eigen::Vector3d ned_t;
        SlamPoseTrans::SlamToLocalpose_test(Tcw,ned_q,ned_t);

        Eigen::Quaterniond rot_q( 0.707 ,0, 0,0.707);
        new_q = ned_q;
        new_t = rot_q* ned_t;
        /*for(int idx=0;idx<Tcw.rows;++idx)
            for(int idy=0;idy<Tcw.cols;++idy)
                msg.Twc[idx*Tcw.rows+idy] = Tcw.at<float>(idx,idy);*/
   /* }
    else
    {
        return;
    }*/
    
    //cv::imshow("0000",left);
    //cv::waitKey(1);
    //std::cout<<bitstream.size()<<" size"<<std::endl;
    msg.x =state_->slam_pos(0);
    msg.y = state_->slam_pos(1);
    msg.z = 3;//state_->mav_pos(2);
    msg.qw = state_->mav_q(0);
    msg.qx = state_->mav_q(1);
    msg.qy = state_->mav_q(2);
    msg.qz = state_->mav_q(3);
    /*msg.x = new_t(0);
    msg.y = new_t(1);
    msg.z = 3;//state_->mav_pos(2);
    msg.qw = new_q.w();
    msg.qx = new_q.x();
    msg.qy = new_q.y();
    msg.qz = new_q.z();*/
    //std::cout << state_->name_ << " current pos " << msg.x << " " << msg.y << " " << msg.z << std::endl;
    
    trans_data_raw += raw_size;
    trans_data+=bitstream.size();
    msg.data.assign(bitstream.begin(), bitstream.end());
    msg.mvuRight.assign(mvuRight.begin(), mvuRight.end());
    msg.mDepth.assign(mvDepth.begin(), mvDepth.end());
    
    other_respons_pubs[id].publish(msg);
}

void MavNavigator::ResponsCallback(const px4_csq::frame_rosPtr msg)
{
    /*if(state_->self_id == 1)
    {
        state_->SetBias(-1,-2,3);
    }
    else
    {
        state_->SetBias(-2,3, 3);
    }
    return;*/
    
    //std::cout<<img_bitstream.size()<<" size"<<std::endl;
    Eigen::Quaterniond msg_q(msg->qw,msg->qx,msg->qy,msg->qz);
    //Eigen::Quaterniond msg_q(-1,0,0,0);
    Eigen::Vector3d msg_t(msg->x,msg->y,msg->z);
    //Eigen::Vector3d msg_t(0,0,3);
    cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
    /*for(int idx=0;idx<pose.rows;++idx)
        for(int idy=0;idy<pose.cols;++idy)
            pose.at<float>(idx,idy)=msg->Twc[idx*pose.rows+idy];*/
    /*std::cout <<"q is "<<new_q.w()<<" "<<new_q.x()<<" "<<new_q.y()<<" "<<new_q.z()<<std::endl;
    std::cout <<"t is "<<msg_t<<std::endl;
    cv::Mat pose2 = cv::Mat::eye(4, 4, CV_32F);*/
    //LocalposeToSlam_test(new_q,msg_t,pose2);
    //std::cout<<"twc is "<<pose2<<std::endl;

    //std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid << std::endl;
    cv::Mat right, left;
    sensors_->GetStereoImage(left, right);
    //cv::imshow("1111",left);
    //cv::waitKey(1);
    //cv::imwrite("/home/jena/csq_ws/wrong_1_0.jpg",left);
    //cv::imwrite("/home/jena/csq_ws/wrong_1_1.jpg",right);
    /*if (!Extractor_init)
    {
        cv::Mat left1(cv::imread("/home/jena/csq_ws/uav0-1_left.jpg", 0));
        cv::Mat right1(cv::imread("/home/jena/csq_ws/uav0-1_right.jpg", 0));
        std::vector<uchar> bitstream1;
        Extractor_init = true;
        coLocal->GenerateFeatureBitstream(1,left1, right1, bitstream1);
    }*/
    std::vector<uchar> bitstream(msg->data.begin(), msg->data.end());
    //test
    /*cv::Mat left1(cv::imread("/home/jena/csq_ws/uav0-2_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/uav0-2_right.jpg", 0));*/
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
    bool insert_keyframe;
    double x_dist = last_colocal_pos[msg->nrobotid](0) -state_->mav_pos(0);
    double y_dist = last_colocal_pos[msg->nrobotid](1) -state_->mav_pos(1);
    if((x_dist*x_dist+y_dist*y_dist)<1)
    {
        insert_keyframe = true;
    }
    else
    {
        insert_keyframe = true;
        last_colocal_pos[msg->nrobotid](0) = state_->mav_pos(0);
        last_colocal_pos[msg->nrobotid](1) = state_->mav_pos(1);
    }
    res = coLocal->TrackFromBitstream(pose,left,right,vDecKeypointsLeft,decDescriptorsLeft,vDecKeypointsRight,decDescriptorsRight,mvuRight,mvDepth,insert_keyframe,msg->nrobotid);
    std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid;
    //res = coLocal2->TrackFromImage(left0,right0,left1,right1,init_pose);
    if (!res.empty())
    {
        Eigen::Quaterniond ned_q;Eigen::Vector3d ned_t;
        SlamPoseTrans::SlamToLocalpose_test(res,ned_q,ned_t);
        Eigen::Quaterniond rot_q( 0.70710678118655  ,0, 0, 0.70710678118655 );
        Eigen::Quaterniond new_q = (ned_q*rot_q*msg_q);
        Eigen::Vector3d new_t = msg_t - ned_q*rot_q* ned_t;
        
        Eigen::Vector3d rel_t = rot_q* ned_t;
        has_communication[msg->nrobotid] = true;
        //std::cout<<" and local position is :"<<msg->x-ned_t(2)<<","<<msg->y+ned_t(0)<<","<<msg->z-ned_t(1)<<","<<std::endl;
        std::cout<<" and new_t is :"<<new_t(0)<<","<<new_t(1)<<","<<new_t(2)<<","<<std::endl;
        std::cout << " and msg position is :" << msg_t(0) << "," << msg_t(1) << "," << msg_t(2) <<  std::endl;
        std::cout << " and slam position is :" << state_->slam_pos(0)<< "," << state_->slam_pos(1)<< "," << rel_t(2) <<  std::endl;
        std::cout << " and rel position is :" << rel_t(0)<< "," << rel_t(1)<< "," << rel_t(2) <<  std::endl;
        //std::cout << " and rel position is :" << rel_t(0) << "," << rel_t(1) << "," << rel_t(2) << "," << std::endl;
        if (msg->nrobotid == 0)
        {
            if (state_->has_colocal_inited == false)
            {
                //state_->SetBias(msg->x-ned_t(2),msg->y+ned_t(0),3);
                state_->SetBias(new_t(0), new_t(1), 3);
                MoveTo(formation_pos(0), formation_pos(1), formation_pos(2), formation_yaw);
                //state_->has_colocal_inited = true;
            }
            else
            {
                state_->SetBias(new_t(0), new_t(1), 3);
                std::cout << "ekf" << std::endl;
            }
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
        //std::cout << "The " << state_->self_id << " get respons from " << msg->nrobotid;
        //std::cout<<" but coloca1 failed"<<std::endl;
    }
}
void MavNavigator::FormationCallback(const px4_csq::colocal_request &msg)
{
    formation_pos(0) = msg.x;
    formation_pos(1) = msg.y;
    formation_pos(2) = msg.z;
    formation_yaw = 0;//msg.yaw;
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
void MavNavigator::DrawTrajThread(void)
{
    while(1)
    {
        for(int i =0;i<3;++i)
        {
            double x_dist = record_pos[i](0) - last_record_pos[i](0);
            double y_dist = record_pos[i](1) - last_record_pos[i](1);
            double z_dist = record_pos[i](2) - last_record_pos[i](2);
            if((x_dist*x_dist+y_dist*y_dist+z_dist*z_dist)<0.01)
            //if(0)
            {
            }
            else
            {
                coLocal->AddTraj(record_pos[i].x(),record_pos[i].y(),record_pos[i].z(),i);
                last_record_pos[i](0) = record_pos[i](0);
                last_record_pos[i](1) = record_pos[i](1);
                last_record_pos[i](2) = record_pos[i](2);
            }
            if(has_communication[i]== true)
            {
                int j = state_->self_id;
                has_communication[i] = false;
                coLocal->AddComm(record_pos[i].x(),record_pos[i].y(),record_pos[i].z(),i,record_pos[j].x(),record_pos[j].y(),record_pos[j].z(),j);
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    }
}
void MavNavigator::RecordThread(void)
{
    char data[1024] ={0};
    struct timeval cur_time;
    while(1)
    {
        if(record_start)
        {
            gettimeofday(&cur_time, NULL);
            long now_time = cur_time.tv_sec*1000000 + cur_time.tv_usec - 1.5e15;
            memset(&data,0,sizeof(data));
            sprintf(data,"%ld\t%d\t%f\t%f\t%f\t%d\t%d\t%f\t%f\t%f\n",now_time,state_->has_colocal_inited, state_->slam_pos(0), state_->slam_pos(1), state_->slam_pos(2),trans_data_raw,trans_data,record_pos[state_->self_id].x(),record_pos[state_->self_id].y(),record_pos[state_->self_id].z());
            recorder->Record(data,0);
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }
}
void MavNavigator::RecordGsThread(void)
{
    char data[1024] ={0};
    struct timeval cur_time;
    while(1)
    {
        if(record_start)
        {
            gettimeofday(&cur_time, NULL);
            long now_time = cur_time.tv_sec*1000000 + cur_time.tv_usec - 1.5e15;
            memset(&data,0,sizeof(data));
            sprintf(data,"%ld\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",now_time,record_pos[0](0),record_pos[0](1),record_pos[0](2),record_pos[1](0),record_pos[1](1),record_pos[1](2),record_pos[2](0),record_pos[2](1),record_pos[2](2));
            gs_recorder->Record(data,0);
           // printf("rel pos of 1 and 2:%f,%f\n",(record_pos[0](0)-record_pos[2](0)),(record_pos[0](1)-record_pos[2](1)));
           printf("pos of 0:%f,%f est pos %f,%f\n",(record_pos[0](0)),(record_pos[0](1)),state_->slam_pos(0),state_->slam_pos(1));
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }
}
void MavNavigator::pos1Callback(const nav_msgs::Odometry &msg)
{
    record_pos[0](0) =  msg.pose.pose.position.x;
    record_pos[0](1) =  msg.pose.pose.position.y;
    record_pos[0](2) =  msg.pose.pose.position.z;
}
void MavNavigator::pos2Callback(const nav_msgs::Odometry &msg)
{
   record_pos[1](0) =  msg.pose.pose.position.x;
   record_pos[1](1) =  msg.pose.pose.position.y;
   record_pos[1](2) =  msg.pose.pose.position.z;
}
void MavNavigator::pos3Callback(const nav_msgs::Odometry &msg)
{
   record_pos[2](0) =  msg.pose.pose.position.x;
   record_pos[2](1) =  msg.pose.pose.position.y;
   record_pos[2](2) =  msg.pose.pose.position.z;
}