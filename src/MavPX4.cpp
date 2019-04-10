#include "MavPX4.h"

MavPX4::MavPX4(const string& id)
{
    name_ = string("/uav")+id;
	control_loop_ =new MavControlLoop(name_);
    state_ = new MavState(name_,control_loop_);
    stereo_name = string("/iris_stereo_camera_")+id;
    sensors_ = new MavSensors(name_,stereo_name,state_);
    navigator_ =new  MavNavigator(state_);
    command_handle_ =new MavCommand(navigator_,state_,sensors_,name_);

}


MavPX4::~MavPX4()
{

}
