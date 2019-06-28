#pragma once

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "MapPoint.h"
#include "Map.h"

#include <thread>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
struct ImageInfo
{
    int cols;
    int rows;
};
class MapPoint;
class Map;

class Frame
{
public:
    // 自己新添加的东西：
    // 飞机的ID，这个添加到构造函数当中。
    int mCraftID;
    bool mbFrameValid;
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
    
    void ExtractORB(int flag, const cv::Mat &im);
    void ComputeCompressedStereoMatches();
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;


    // Returns the camera center.
    inline cv::Mat GetCameraCenter()
	{
        return mOw.clone();
    }

    
    int N; //特征点个数96

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    static bool mbInitialComputations;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    std::vector<bool> mvbOutlier;
    std::vector<MapPoint*> mvpMapPoints;

    Map* mpMap;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    // 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
    // FRAME_GRID_ROWS 48
    // FRAME_GRID_COLS 64
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw; ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵

    // Current and Next Frame id.
    static long unsigned int nNextId; ///< Next Frame id.
    long unsigned int mnId;           ///< Current Frame id.

    // Scale pyramid info.
    int mnScaleLevels;      //图像提金字塔的层数
    float mfScaleFactor;    //图像提金字塔的尺度因子
    float mfLogScaleFactor; //
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    // 用于确定画格子时的边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    // Rotation, translation and camera center
    cv::Mat mRcw; ///< Rotation from world to camera
    cv::Mat mtcw; ///< Translation from world to camera
    cv::Mat mRwc; ///< Rotation from camera to world
    cv::Mat mOw;  ///< mtwc,Translation from camera to world

    /***********************以下为KeyFrame专用*****************************/
    //BoW
    DBoW2::BowVector mBowVec; ///< Vector of words to represent images
    DBoW2::FeatureVector mFeatVec; ///< Vector of nodes with indexes of local features

    // BoW
    // KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    std::vector<MapPoint*> GetMapPointMatches();

    // SE3 Pose and camera center
    // cv::Mat Tcw;
    // cv::Mat Twc;
    // cv::Mat Ow;

    /***********************以下为KeyFrame专用 结束*************************/
public:    
    Frame();
    //根据特征和匹配关系生成帧
    Frame(const std::vector<cv::KeyPoint> &keyPointsLeft, const cv::Mat &descriptorLeft,
             const std::vector<cv::KeyPoint> &keyPointsRight,
             const cv::Mat &descriptorRight, ORBextractor *extractorLeft,
             ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &dist, const float &baseline,
             const float &thDepth, ImageInfo Info, const std::vector<float> &vuRight, const std::vector<float> &vDepth);
    Frame(const cv::Mat &imLeft, 
    const cv::Mat &imRight, 
    ORBextractor *extractorLeft, 
    ORBextractor *extractorRight, 
    cv::Mat &K,cv::Mat &dist, ORBVocabulary* voc,
    float baseline, 
    int craft_id,ImageInfo Info);
    Frame(const std::vector<cv::KeyPoint> &keyPointsLeft, const cv::Mat &descriptorLeft,
             const std::vector<unsigned int> &visualWords, const std::vector<cv::KeyPoint> &keyPointsRight,
             const cv::Mat &descriptorRight, ORBextractor *extractorLeft,
             ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &dist, const float &baseline,
             const float &thDepth,ImageInfo Info);
    ~Frame()
    {
    }

    void ComputeStereoMatches();
    void trace(std::vector<float> vec);
    void AddMapPoint(MapPoint* pMP, const size_t &idx);

    // Set the camera pose.
    // 用Tcw更新mTcw
    void SetPose(cv::Mat Tcw);   
    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();
    cv::Mat UnprojectStereo(int i);
    
    void ComputeBoW();
    void ComputeImageBounds(const cv::Mat &imLeft);
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    
    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    // mvKeys:原始左图像提取出的特征点（未校正）
    // mvKeysRight:原始右图像提取出的特征点（未校正）
    // mvKeysUn:校正mvKeys后的特征点，对于双目摄像头，一般得到的图像都是校正好的，再校正一次有点多余
    
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinat96e and depth for each keypoint.
    // "Monocular" keypoints have a n96egative value.
    // 对于双目，mvuRight存储了左目像96素点在右目中的对应点的横坐标
    // mvDepth对应的深度96
    // 单目摄像头，这两个容器中存的都96是-1
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;
    std::vector<int> mvDist;
    cv::Mat mDescriptors, mDescriptorsRight;
    std::vector<DMatch> matches;
};
