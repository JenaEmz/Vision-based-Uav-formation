#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <iostream>
#include <mutex>
#include <sys/socket.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Tracking.h"
#include "feature_coder.h"

using namespace std;
enum ImageType
{
    OpenCv = 0,
    RosCallback
};

class CoLocalSystem
{
  public:
    Tracking *GetTracker();
    CoLocalSystem(const cv::FileStorage &fsSettings);
    ~CoLocalSystem();

    bool GenerateLastFrame(cv::Mat &imLeft, cv::Mat &imRight);
    cv::Mat TrackFromGenerate(cv::Mat& init,cv::Mat &imLeft, cv::Mat &imRight,const std::vector<cv::KeyPoint> &keyPointsLeft, const cv::Mat &descriptorLeft,
                                        const std::vector<cv::KeyPoint> &keyPointsRight,const cv::Mat &descriptorRight,
                                         const std::vector<float> &mvuRight, const std::vector<float> &mvDepth);
    bool SetCurrentFrame(cv::Mat &imLeft, cv::Mat &imRight);
    cv::Mat TrackFromImage(cv::Mat &imLeft0, cv::Mat &imRight0, cv::Mat &imLeft1, cv::Mat &imRight1,cv::Mat& init);
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, int id);
    void GenerateFeatureBitstream(int id,const cv::Mat &imLeft, const cv::Mat &imRight, std::vector<uchar> &bitstream);
    cv::Mat TrackFromBitstream(std::vector<uchar> &img_bitstream, cv::Mat &Tcw, const cv::Mat &imLeft, const cv::Mat &imRight);
   

    void ExtractORB(int flag, const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys, cv::Mat &descriptors);
    //params* mParams;
    cv::Mat TrackFromKeypoint(cv::Mat &Tcw, std::vector<cv::KeyPoint> &keyPointsLeft,
                           cv::Mat &descriptorLeft, std::vector<cv::KeyPoint> &keyPointsRight,
                           cv::Mat &descriptorRight,
                           const cv::Mat &imLeft, const cv::Mat &imRight);
    LBFC2::FeatureCoder *mEncoder;
    LBFC2::FeatureCoder *mDecoder;
    Tracking *mTracker;
  private:
    ros::NodeHandle nh;
    cv::FileStorage fsSettings_;

    LBFC2::CodingStats codingModel;
    ORBVocabulary mpVocabulary;
    void Encoder_init(const cv::FileStorage &fSettings);

    cv::Mat mImRight, mImLeft;
    cv::Mat M1l, M2l, M1r, M2r;

    std::mutex mImMutex;
    std::mutex coderMutex;

    int listen_fd;
};

/*struct params
{
    //camera_param
    float fx;
    float fy;
    float cx;
    float cy;
    int imgWidth;
	int imgHeight;

    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4, 1, CV_32F);
    float mbf;

    int debug = 1;
    // 每一帧提取的特征点数 1000
    int nFeatures ;
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor;
    // 尺度金字塔的层数 8
    int nLevels;
    // 提取fast特征点的默认阈值 20
    int fIniThFAST;
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST;

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    bool bVocLoad ;// chose loading method based on file extension
    string dictPath;
}*/