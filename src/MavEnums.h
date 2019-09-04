#ifndef MAV_ENUMS
#define MAV_ENUMS
#include <deque>
//保存枚举变量
enum ControlType
{
    NOT_CONTROL = 0,
    NORMAL_POSITION,
    NORMAL_VELOCITY,
    VISION_FORMATION
};
enum CommandType
{
    ARMING = 0,
    TAKEINGOFF,
    LANDING,
    MOVING,
    MISSION,
    FORMATION
};
enum MissionType
{
    TAKEOFF,
    LAND,
    MOVE_TO,
    HOLD,
    FORMATE
};
struct MissionPoint
{
    MissionType type;
    double x;
    double y;
    double z;
    double yaw;
    bool start;
};
typedef std::deque<MissionPoint> MissionTrajector;
#endif ///*define*/


