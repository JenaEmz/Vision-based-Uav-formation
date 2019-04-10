#ifndef MAV_NAVIGATOR
#define MAV_NAVIGATOR

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Core>

#include "MavState.h"
#include "MavEnums.h"

class MavNavigator
{
  public:
    MavNavigator(MavState *state);
    ~MavNavigator();

    void Takeoff(double x, double y, double height);
    void Land(double x, double y);
    void Hover();
    void MoveTo(double x,double y,double z,double yaw);
    void Hold(double x,double y,double z,double yaw);
    bool Mission(MissionPoint &Mission);

  private:
    MavState *state_;
};

inline double Distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

#endif //MAV_STATE
