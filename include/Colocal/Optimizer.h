#pragma once

#include "Map.h"
#include "MapPoint.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

class Optimizer{    
public:
    int static PoseOptimization(Frame* pFrame);
};