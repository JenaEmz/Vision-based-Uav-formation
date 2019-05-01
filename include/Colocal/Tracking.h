#pragma once

#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Frame.h"
#include "MapPoint.h"

#include "Map.h"
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <thread>
using namespace std;

class Map;

class Tracking
{
  private:
    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    Frame mCurrentFrame;
    Frame mLastFrame;
    int mTrackFlag;
    float mThDepth;
    Map* mpMap;
    int cols;
    int rows;

    int debug;

    cv::Mat out_left_1;
    cv::Mat out_right_1;
    cv::Mat out_left_2;
    cv::Mat out_right_2;

    ORBVocabulary* mpVocabulary;

  public:

    cv::Mat show;
    void traceMap(Frame* frame);
    void draw_match(vector<MapPoint* > mvpMapPoints);
    void StereoInitialization();

    //new function
    bool StereoInitialization(cv::Mat& Tcw);
    bool TrackStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight);
    //set last frame
    bool TrackStereoBitstream(cv::Mat& Tcw, const std::vector<cv::KeyPoint> &keyPointsLeft,
                                       const cv::Mat &descriptorLeft, const std::vector<unsigned int> &visualWords, 
                                       const std::vector<cv::KeyPoint> &keyPointsRight, const cv::Mat &descriptorRight);
    Tracking(const cv::FileStorage &fSettings);
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, int id);
    bool LoopDetecting();
    bool RelativePoseEPnp();
    bool RelativePoseG2o();
    bool SetCurrentFrame(const cv::Mat &imRectLeft, const cv::Mat &imRectRight);
    cv::Mat GetRelPose();
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
};
