#include "MavCommand.h"

MavCommand::MavCommand(MavNavigator *navigator, MavState *state,MavSensors* sensors,const string& name) : navigator_(navigator), state_(state),sensors_(sensors)
{
    //cmd_thread_ = std::thread(&MavCommand::CommandFromStringThread,this);
    name_ = name;
    gs_cmd_sub_ = nh_.subscribe(name_ + "/gs_cmd", 1, &MavCommand::CommandFromTopic, this);
}

MavCommand::~MavCommand()
{

}
void MavCommand::CommandFromTopic(const px4_csq::gs_command& msg)
{
    switch(msg.mission_type)
    {
        case 0:
            mission_quit = true;
            state_->MavEnableControl();
            break;
        case 1:
             mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::TAKEOFF,0,0,3,0,false});
            
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
            break;
        case 2:
            mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,msg.x,msg.y,msg.z,msg.yaw,false});
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
        case 3:
            state_->UpdateBias();
            std::cout<<"init state:"<<state_->has_colocal_inited;
    }
}
void MavCommand::CommandFromStringThread()
{
    //string input_string;
    char ch;
    while (ros::ok())
    {
        cout << "Please enter a char : "<<endl;
        ch = '0';
        std::cin.get(ch);
        switch (ch)
        {
        case 49: //1 //启动飞机
        {
            mission_quit = true;
            state_->MavEnableControl();
            break;
        }
        break;
        case 50: //2
            mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::TAKEOFF,0,0,3,0,false});
            
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
            break;
        case 51:
         mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0,0,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,1,0,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,-1,0,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,1,0,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0,0,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0.5,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,-0.5,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0.5,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,-0.5,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1.3,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,0.7,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1.3,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,0.7,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1,0.1,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1.2,-0.1,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0.3,1.2,-0.1,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1,0.1,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0.2,1.2,-0.1,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0.5,0,1,0.1,false});
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
            break;
        case 52: //4
            mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0,1,2,1.57,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,1,1,2,1.57,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,1,3,2,1.57,false});
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
            break;
         case 53: //5
            mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,1,1,2,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,1,-1,2,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,-1,-1,2,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,-1,1,2,0,false});
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,0,0,2,0,false});
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
            break;
         case 54: //6
            sensors_->RecordImgOnce();
            break;
        }
    }
}

void MavCommand::MissionThread()
{
    while(state_->MavIsOk()&&!mission_tarjector_.empty()&&!mission_quit)
    {
        mission_mtx.lock();
        if(navigator_->Mission(mission_tarjector_.front()))
        {
            cout<<"complete mission "<<mission_tarjector_.front().type<<endl;
            mission_tarjector_.pop_front();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
        }
        mission_mtx.unlock();
    }
}