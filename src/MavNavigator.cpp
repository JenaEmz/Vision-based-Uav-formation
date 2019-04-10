#include "MavNavigator.h"

MavNavigator::MavNavigator(MavState *state) : state_(state)
{
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