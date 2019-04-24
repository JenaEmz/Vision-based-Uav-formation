
#pragma once

#include"Frame.h"
#include"Map.h"

#include <map>
#include<opencv2/core/core.hpp>


class Map;
class Frame;

class MapPoint{
public:    
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);
public:    
    bool mbBad;    
    bool isBad();
    cv::Mat GetDescriptor();
    // Mean viewing direction
    // 该MapPoint平均观测方向
    cv::Mat mNormalVector;

    int Observations();

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    // Reference KeyFrame
    Frame* mpRefKF;

    // ORB descriptor, each row associated to a keypoint.
    // 左目摄像头和右目摄像头特征点对应的描述子
    cv::Mat mDescriptors, mDescriptorsRight;

    static long unsigned int nNextId;
    long unsigned int mnId; ///< Global ID for MapPoint

    // Best descriptor to fast matching
    // 每个3D点也有一个descriptor
    // 如果MapPoint与很多帧图像特征点对应（由keyframe来构造时），那么距离其它描述子的平均距离最小的描述子是最佳描述子
    // MapPoint只与一帧的图像特征点对应（由frame来构造时），那么这个特征点的描述子就是该3D点的描述子
    cv::Mat mDescriptor; ///< 通过 ComputeDistinctiveDescriptors() 得到的最优描述子
    
    // Keyframes observing the point and associated index in keyframe
    std::map<Frame*,size_t> mObservations; ///< 观测到该MapPoint的KF和该MapPoint在KF中的索引
    int nObs;

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();
    // Position in absolute coordinates
    cv::Mat mWorldPos; ///< MapPoint在世界坐标系下的坐标

    void ComputeDistinctiveDescriptors();
    void UpdateNormalAndDepth();
    void AddObservation(Frame* pKF,size_t idx);
};