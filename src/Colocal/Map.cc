
#include "Map.h"
Map::Map():mnMaxKFid(0){
    
}

/**
 * @brief Insert Frame in the map
 * @param pF Frame
 */
void Map::AddFrame(Frame *pF)
{
    // unique_lock<mutex> lock(mMutexMap);
    mspFrames.insert(pF);
    if(pF->mnId>mnMaxKFid)
        mnMaxKFid=pF->mnId;
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseFrame(Frame *pF)
{
    // unique_lock<mutex> lock(mMutexMap);
    mspFrames.erase(pF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
void Map::AddMapPoint(MapPoint *pMP)
{
    // unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}


/**
 * @brief Erase MapPoint from the map
 * @param pMP MapPoint
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    // unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    delete pMP;
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}