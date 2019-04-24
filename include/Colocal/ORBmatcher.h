#pragma once

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"Frame.h"

class ORBmatcher{
public:
    float mfNNratio;
    bool mbCheckOrientation;
    ORBmatcher(float nnratio=0.6, bool checkOri=true);
    
    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    /**
     * @brief 通过词包，对关键帧的特征点进行跟踪
     * 
     * KeyFrame中包含了MapPoints，对这些MapPoints进行tracking \n
     * 由于每一个MapPoint对应有描述子，因此可以通过描述子距离进行跟踪 \n
     * 为了加速匹配过程，将关键帧和当前帧的描述子划分到特定层的nodes中 \n
     * 对属于同一node的描述子计算距离进行匹配 \n
     * 通过距离阈值、比例阈值和角度投票进行剔除误匹配
     * @param  pKF               KeyFrame
     * @param  F                 Current Frame
     * @param  vpMapPointMatches F中MapPoints对应的匹配，NULL表示未匹配
     * @return                   成功匹配的数量
     */
    int SearchByBoW(Frame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches, vector<int>& rotHist);
    void ComputeThreeMaxima(std::vector<std::vector<int> > histo, const int L, int &ind1, int &ind2, int &ind3);

    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
    void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);
public:
    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;
};