#include "MavNavigator.h"

MavNavigator::MavNavigator(MavState *state,MavSensors* sensors,const string& config_path) : state_(state),sensors_(sensors)
{
    fsSettings_ = cv::FileStorage(config_path, cv::FileStorage::READ);
    coLocal = new CoLocalSystem(fsSettings_);
       //多机通信部分
    request_sub = nh_.subscribe("/request_list",10,&MavNavigator::RequestCallback,this);
    request_pub = nh_.advertise<px4_csq::colocal_request>("/request_list", 10);
    self_respons_sub =  nh_.subscribe(state->name_+"/Respons",10,&MavNavigator::ResponsCallback,this);
    
    for(int i=0;i<state_->total_num;i++)
    {
        string bitstreamTopic = "/uav"+to_string(i)+"/Respons";
        other_respons_pubs.emplace_back(nh_.advertise<orb_formation::feature>(bitstreamTopic, 1000, true)); 
    }
}

MavNavigator::~MavNavigator()
{
}

void MavNavigator::Takeoff(double x, double y, double height = 4)
{
    state_->set_pos_sp(x, y, height);
    state_->set_control_mode( NORMAL_POSITION );
}
void MavNavigator::Land(double x, double y)
{
    state_->set_pos_sp(x, y, -1.0);
    state_->set_control_mode( NORMAL_POSITION );
}
void MavNavigator::Hover()
{
    state_->set_pos_sp(state_->get_pos(0), state_->get_pos(1), state_->get_pos(2));
    state_->set_control_mode( NORMAL_POSITION );
}
void MavNavigator::MoveTo(double x, double y, double z, double yaw)
{
    state_->set_pos_sp(x,y,z);
    state_->set_yaw_sp(yaw);
    state_->set_control_mode( NORMAL_POSITION );
}
void MavNavigator::Hold(double x, double y, double z, double yaw)
{
    state_->set_pos_sp(x,y,z);
    state_->set_yaw_sp(yaw);
    state_->set_control_mode( NORMAL_POSITION );
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
        MoveTo(Mission.x, Mission.y, Mission.z,Mission.yaw);
        if (Distance(Mission.x, Mission.y, state_->get_pos(0), state_->get_pos(1)) < 0.2&& (abs(Mission.z - state_->get_pos(2)) < 0.1))
            return true;
        else
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
        Hold(Mission.x, Mission.y, Mission.z,Mission.yaw);
        return false;
        break;
    }
}

void MavNavigator::UpdateBias()
{
    px4_csq::colocal_request msg;
    msg.has_inited = state_->has_colocal_inited;
    msg.x =  state_->mav_pos(0);
    msg.y =  state_->mav_pos(1);
    msg.z =  state_->mav_pos(2);
    msg.id = state_->self_id;
    if(state_->self_id == 0)return;
    request_pub.publish(msg);

    //测试位姿估计
    /*cv::Mat left0(cv::imread("/home/jena/csq_ws/src/orb_formation/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/src/orb_formation/uav0-1_right.jpg", 0));
    cv::Mat left1(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_right.jpg", 0));
    cv::Mat res;
    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    coLocal->TrackFromImage(left0,right0,left1,right1);*/
    
    /*cv::Mat left0(cv::imread("/home/jena/csq_ws/src/orb_formation/uav0-1_left.jpg", 0));
    cv::Mat right0(cv::imread("/home/jena/csq_ws/src/orb_formation/uav0-1_right.jpg", 0));

    //测试 压缩和解压特征点
    std::vector<uchar> bitstream;
    coLocal->GenerateFeatureBitstream(left0,right0,bitstream);
    orb_formation::feature msg;
    msg.data.assign(bitstream.begin(),bitstream.end());
    std::vector<uchar> img_bitstream(msg.data.begin(), msg.data.end());

    cv::Mat left1(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_left.jpg", 0));
    cv::Mat right1(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_right.jpg", 0));
    cv::Mat res;
    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    
    //coLocal.TrackStereo(left0,right0,0);
    //std::cout<< coLocal.TrackStereo(left1,right1,1);
    res = coLocal->TrackFromBitstream(bitstream,init_pose,left1,right1);
    std::cout<<res<<std::endl;*/
}
void MavNavigator::RequestCallback(const px4_csq::colocal_request &msg)
{
    if(msg.id == state_->self_id||state_->has_colocal_inited==false)return;
    if(msg.has_inited == false)
    {
        std::cout<<"the"<< state_->self_id<<"get request from "<<msg.id <<std::endl;
        PubRespons(msg.id);
    }
    else
    {
        //计算共视角先

        //
    }
}

void MavNavigator::ResponsCallback(const orb_formation::feature::ConstPtr msg)
{
    std::vector<uchar> img_bitstream(msg->data.begin(), msg->data.end());
    //std::cout<<img_bitstream.size()<<" size"<<std::endl;
	std::vector<cv::KeyPoint> vDecKeypointsLeft, vDecKeypointsRight;
    cv::Mat decDescriptorsLeft, decDescriptorsRight;

    orb_formation::feature msg1;
    msg1.x = state_->mav_pos(0);
    msg1.y = state_->mav_pos(1);
    msg1.z = state_->mav_pos(2);
    std::cout<<state_->name_<<" current pos "<<msg1.x<<" "<<msg1.y<<" "<<msg1.z<<std::endl;

    cv::Mat init_pose = cv::Mat::eye(4, 4, CV_32F);
    std::cout<<"The "<< state_->self_id<<" get respons from "<<msg->nrobotid <<std::endl;
    cv::Mat right,left;
    cv::Mat res;
    sensors_->GetStereoImage(left,right);
    
    if(!Extractor_init)
    {
        cv::Mat left1(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_left.jpg", 0));
        cv::Mat right1(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_right.jpg", 0));
        std::vector<uchar> bitstream;
        Extractor_init =true;
        coLocal->GenerateFeatureBitstream(left1,right1,bitstream);
    }
    //res = coLocal->TrackFromBitstream(img_bitstream,init_pose,left1,right1);
    res = coLocal->TrackFromBitstream(img_bitstream,init_pose,left,right);
    if(!res.empty())
    {
        std::cout<<"colocal success:"<<res<<std::endl;
    }
    else
    {
        //res = coLocal->TrackFromBitstream(img_bitstream,init_pose,left,right);
    }
    
}

void MavNavigator::PubRespons(int id)
{
    orb_formation::feature msg;
    msg.nrobotid = state_->self_id;
    cv::Mat right,left;
    std::vector<uchar> bitstream;
    std::cout<<"Pub respons from:"<<state_->self_id<<std::endl;
    sensors_->GetStereoImage(left,right);
    coLocal->GenerateFeatureBitstream(left,right,bitstream);
    //cv::Mat left0(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_left.jpg", 0));
    //cv::Mat right0(cv::imread("/home/jena/csq_ws/src/orb_formation/uav1-1_right.jpg", 0));
    //oLocal->GenerateFeatureBitstream(left0,right0,bitstream);

    //std::cout<<bitstream.size()<<" size"<<std::endl;
    msg.x = state_->mav_pos(0);
    msg.y = state_->mav_pos(1);
    msg.z = state_->mav_pos(2);
    msg.qw = state_->mav_q(0);
    msg.qx = state_->mav_q(0);
    msg.qy = state_->mav_q(1);
    msg.qz = state_->mav_q(2);
    std::cout<<state_->name_<<" current pos "<<msg.x<<" "<<msg.y<<" "<<msg.z<<std::endl;
    msg.data.assign(bitstream.begin(),bitstream.end());
    other_respons_pubs[id].publish(msg);
}