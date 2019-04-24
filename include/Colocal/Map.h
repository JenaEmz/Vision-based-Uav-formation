#pragma once

#include "MapPoint.h"
#include "Frame.h"

#include <set>
class Frame;
class MapPoint;

class Map{
public:
    Map();
    void AddFrame(Frame* pF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseFrame(Frame* pF);

    std::set<MapPoint*> mspMapPoints; ///< MapPoints
    std::set<Frame*> mspFrames; ///< Frames

    long unsigned int mnMaxKFid;
};