
#include "MapPoint.h"

long unsigned int MapPoint::nNextId=0;

bool MapPoint::isBad(){
    return mbBad;
}

/**
 * @brief 给定坐标与frame构造MapPoint
 *
 * 双目：UpdateLastFrame()
 * @param Pos    MapPoint的坐标（wrt世界坐标系）
 * @param pMap   Map
 * @param pFrame Frame
 * @param idxF   MapPoint在Frame中的索引，即对应的特征点的编号
 */
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mpRefKF(static_cast<Frame*>(NULL)), mbBad(false)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;// 世界坐标系下相机到3D点的向量
    mNormalVector = mNormalVector/cv::norm(mNormalVector);// 世界坐标系下相机到3D点的单位向量
    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);

    const int level = pFrame->mvKeysUn[idxF].octave;
    
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];

    const int nLevels = pFrame->mnScaleLevels;
    // 另见PredictScale函数前的注释
    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];
    // 见mDescriptor在MapPoint.h中的注释
    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    mnId=nNextId++;
    mpRefKF = pFrame;
}
int MapPoint::Observations()
{
    return nObs;
}
/**
 * @brief 计算具有代表的描述子
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要判断是否更新当前点的最适合的描述子 \n
 * 先获得当前点的所有描述子，然后计算描述子之间的两两距离，最好的描述子与其他描述子应该具有最小的距离中值
 * @see III - C3.3
 */
cv::Mat MapPoint::GetDescriptor()
{
    return mDescriptor.clone();
}
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<Frame*,size_t> observations;

    {
        // unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        // 这里观测的只有一帧图像,所以观测向量就只有1个了。
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    // 遍历观测到3d点的所有关键帧，获得orb描述子，并插入到vDescriptors中
    for(map<Frame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        Frame* pKF = mit->first;
        vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }
    
    if(vDescriptors.empty())
        return;
        
    {
        // unique_lock<mutex> lock(mMutexFeatures);
        
        // 最好的描述子，该描述子相对于其他描述子有最小的距离中值
        // 简化来讲，中值代表了这个描述子到其它描述子的平均距离
        // 最好的描述子就是和其它描述子的平均距离最小
        mDescriptor = vDescriptors[0].clone();       
    }
}


/**
 * @brief 更新平均观测方向以及观测距离范围
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
 * @see III - C2.2 c2.4
 */
void MapPoint::UpdateNormalAndDepth()
{
    map<Frame*,size_t> observations;
    Frame* pKF;
    cv::Mat Pos;
    {
        // unique_lock<mutex> lock1(mMutexFeatures);
        // unique_lock<mutex> lock2(mMutexPos);
        // if(mbBad)
        //     return;

        observations=mObservations; // 获得观测到该3d点的所有关键帧
        pKF = mpRefKF;                // 观测到该点的参考关键帧
        Pos = mWorldPos.clone();    // 3d点在世界坐标系中的位置
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<Frame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        Frame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali); // 对所有关键帧对该点的观测方向归一化为单位向量进行求和
        n++;
    } 

    cv::Mat PC = Pos - pKF->GetCameraCenter(); // 参考关键帧相机指向3D点的向量（在世界坐标系下的表示）
    const float dist = cv::norm(PC); // 该点到参考关键帧相机的距离
    const int level = pKF->mvKeysUn[observations[pKF]].octave;
    const float levelScaleFactor =  pKF->mvScaleFactors[level];
    const int nLevels = pKF->mnScaleLevels; // 金字塔层数

    {
        //unique_lock<mutex> lock3(mMutexPos);
        // 另见PredictScale函数前的注释
        mfMaxDistance = dist*levelScaleFactor;                           // 观测到该点的距离下限
        mfMinDistance = mfMaxDistance/pKF->mvScaleFactors[nLevels-1]; // 观测到该点的距离上限
        mNormalVector = normal/n;                                        // 获得平均的观测方向
    }
}

/**
 * @brief 添加观测
 *
 * 记录哪些KeyFrame的那个特征点能观测到该MapPoint \n
 * 并增加观测的相机数目nObs，单目+1，双目或者grbd+2
 * 这个函数是建立关键帧共视关系的核心函数，能共同观测到某些MapPoints的关键帧是共视关键帧
 * @param pKF KeyFrame
 * @param idx MapPoint在KeyFrame中的索引
 */
void MapPoint::AddObservation(Frame* pKF, size_t idx)
{
    // unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    // 记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
    mObservations[pKF]=idx;
    // if(pKF->mvuRight[idx]>=0)
    nObs+=2; // 双目或者grbd
    // else
    //     nObs++; // 单目
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    // unique_lock<mutex> lock2(mGlobalMutex);
    // unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    // unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}