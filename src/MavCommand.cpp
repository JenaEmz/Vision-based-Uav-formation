#include "MavCommand.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

MavCommand::MavCommand(MavNavigator *navigator, MavState *state,MavSensors* sensors,const string& name) : navigator_(navigator), state_(state),sensors_(sensors)
{
    cmd_thread_ = std::thread(&MavCommand::CommandFromStringThread,this);
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
            break;
        case 3:
            navigator_->UpdateBias();
            std::cout<<"init state:"<<state_->has_colocal_inited<<std::endl;
            break;
        case 4:
            mission_quit = true;
            if (mission_thread_.joinable())
                mission_thread_.join();
            mission_mtx.lock();
            mission_tarjector_.clear();
            mission_mtx.unlock();
            mission_tarjector_.push_back(MissionPoint{MissionType::MOVE_TO,msg.x,msg.y,msg.z,msg.yaw,false});
            mission_quit = false;
            mission_thread_ = std::thread(&MavCommand::MissionThread,this);
            break;
        case 10:
            sensors_->RecordImgOnce();
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
         case 'j':
            bool keep_control = true;
            while(keep_control)
            {
                if(kbhit())
                {
                    char key;
                    std::cin.get(key);
                    double x = state_->get_pos_sp(0),y = state_->get_pos_sp(1),z = state_->get_pos_sp(2);
                    double yaw = state_->get_yaw_sp();
                    float speed = 0.05;
                    printf("%f,%f,%f,%f\n",x,y,z,yaw);
                    switch(key)
                    {
                        case 'k':
                        keep_control = false;
                        break;
                        case 'w':
                        state_->set_pos_sp(x+speed,y,z);
                        break;
                        case 's':
                        state_->set_pos_sp(x-speed,y,z);
                        break;
                        case 'a':
                        state_->set_pos_sp(x,y-speed,z);
                        break;
                        case 'd':
                        state_->set_pos_sp(x,y+speed,z);
                        break;
                        case 'z':
                        state_->set_pos_sp(x,y,z+speed);
                        break;
                        case 'x':
                        state_->set_pos_sp(x,y,z-speed);
                        break;
                        case 'q':
                        state_->set_yaw_sp(yaw+speed);
                        break;
                        case 'e':
                        state_->set_yaw_sp(yaw-speed);
                        break;
                    }
                }
            }
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
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}