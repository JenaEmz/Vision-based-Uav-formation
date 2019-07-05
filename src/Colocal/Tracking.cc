#include "Tracking.h"

#include "Optimizer.h"

#include "PnPsolver.h"

Tracking::Tracking(const cv::FileStorage &fSettings) : mTrackFlag(0)
{
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);
    cols = fSettings["Camera.width"];
    rows = fSettings["Camera.height"];
    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);
    mbf = fSettings["Camera.bf"];

    debug = 1;
    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈值 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    string dictPath = fSettings["DictPath"];
    bVocLoad = mpVocabulary->loadFromBinaryFile(dictPath);
    mpMap = new Map();

    if (!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << dictPath << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl
         << endl;

    // 判断一个3D点远/近的阈值 mbf * 35 / fx
    mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
    cout << endl
         << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, int id)
{
    float bf = mbf;

    ImageInfo info;
    info.cols = cols;
    info.rows = rows;
    if (id == 0) //是自己的，也就是参考帧。
    {
        mLastFrame = Frame(imRectLeft, imRectRight,
                           mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef,
                           mpVocabulary, bf, id, info);
        imRectLeft.copyTo(out_left_1);
        imRectRight.copyTo(out_right_1);
        cv::cvtColor(out_left_1, out_left_1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(out_right_1, out_right_1, cv::COLOR_GRAY2BGR);
        mLastFrame.mbFrameValid = true;
        StereoInitialization();
        return cv::Mat();
    }
    else
    {
        imRectLeft.copyTo(out_left_2);
        imRectRight.copyTo(out_right_2);
        cv::cvtColor(out_left_2, out_left_2, cv::COLOR_GRAY2BGR);
        cv::cvtColor(out_right_2, out_right_2, cv::COLOR_GRAY2BGR);
        // 别人的帧
        mCurrentFrame = Frame(imRectLeft, imRectRight,
                              mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef,
                              mpVocabulary, bf, id, info);
        mCurrentFrame.mbFrameValid = true;
        //RelativePoseG2o();
        mCurrentFrame.ComputeBoW();
        mLastFrame.ComputeBoW();
        cv::Mat res = cv::Mat();
        bool success = false;
        if (LoopDetecting())
        {
            success = RelativePoseG2o();
            res = mCurrentFrame.mTcw.clone();
            std::cout << "loop detected;" << std::endl;
        }
        else
            res = cv::Mat();
        mCurrentFrame.mbFrameValid = false;
        mLastFrame.mbFrameValid = false;
        if (success)
        {

            //cout << "relative pose is:\n" << res << endl;
            return res;
        }
        return cv::Mat();
    }
}
/*
 * @由压缩特征得到Frame
*/
bool Tracking::TrackStereoBitstream(cv::Mat &Tcw, const std::vector<cv::KeyPoint> &keyPointsLeft,
                                    const cv::Mat &descriptorLeft, const std::vector<unsigned int> &visualWords,
                                    const std::vector<cv::KeyPoint> &keyPointsRight, const cv::Mat &descriptorRight)
{
    ImageInfo info;
    info.cols = cols;
    info.rows = rows;
    mLastFrame = Frame(keyPointsLeft, descriptorLeft, visualWords, keyPointsRight, descriptorRight,
                       mpORBextractorLeft, mpORBextractorRight, mpVocabulary, mK, mDistCoef, mbf, mThDepth, info);
 
    mLastFrame.mbFrameValid = StereoInitialization(Tcw);
    return mLastFrame.mbFrameValid;
}

bool Tracking::GenerateLastFrame(const cv::Mat &imRectLeft, const cv::Mat &imRectRight)
{
    ImageInfo info;
    info.cols = cols;
    info.rows = rows;
    mLastFrame = Frame(imRectLeft, imRectRight,
                       mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef,
                       mpVocabulary, mbf, 0, info);
    //cv::Mat init = cv::Mat::eye(4, 4, CV_32F);
    return true;
}
bool Tracking::RegenerateLastFrame(cv::Mat &Tcw, const std::vector<cv::KeyPoint> &keyPointsLeft, const cv::Mat &descriptorLeft,
                                   const std::vector<cv::KeyPoint> &keyPointsRight, const cv::Mat &descriptorRight,
                                   const std::vector<float> &mvuRight, const std::vector<float> &mvDepth)
{
    ImageInfo info;
    info.cols = cols;
    info.rows = rows;
    mLastFrame = Frame(keyPointsLeft, descriptorLeft, keyPointsRight, descriptorRight, mpORBextractorLeft,mpORBextractorRight,
                       mpVocabulary, mK, mDistCoef, mbf, 0, info, mvuRight, mvDepth);
    
    mLastFrame.mbFrameValid = StereoInitialization(Tcw);
    //cv::Mat init = cv::Mat::eye(4, 4, CV_32F);
    return mLastFrame.mbFrameValid;
}
bool Tracking::TrackStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight,cv::Mat& init)
{
    ImageInfo info;
    info.cols = cols;
    info.rows = rows;
    mLastFrame = Frame(imRectLeft, imRectRight,
                       mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef,
                       mpVocabulary, mbf, 0, info);
    //cv::Mat init = cv::Mat::eye(4, 4, CV_32F);
    mLastFrame.mbFrameValid = StereoInitialization(init);
    return mLastFrame.mbFrameValid;
}

bool Tracking::SetCurrentFrame(const cv::Mat &imRectLeft, const cv::Mat &imRectRight)
{
    ImageInfo info;
    info.cols = cols;
    info.rows = rows;
    show = imRectLeft.clone();
    mCurrentFrame = Frame(imRectLeft, imRectRight,
                          mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef,
                          mpVocabulary, mbf, 0, info);

    mCurrentFrame.mbFrameValid = mCurrentFrame.N > 300;

    return mCurrentFrame.mbFrameValid;
}
cv::Mat Tracking::GetRelPose()
{
    mCurrentFrame.ComputeBoW();
    mLastFrame.ComputeBoW();
    cv::Mat res = cv::Mat();
    bool success = false;
    if (LoopDetecting())
    {
        std::cout << "loop detected;" << std::endl;
        //success = RelativePoseEPnp();
        //if (!success)
        //{
            success = RelativePoseG2o();
            //std::cout << "PnP failed;" << std::endl;
        //}
        res = mCurrentFrame.mTcw.clone();
    }
    else
        res = cv::Mat();
    mCurrentFrame.mbFrameValid = false;
    mLastFrame.mbFrameValid = false;

    if (success)
    {
        return res;
    }
    else
    {
        return cv::Mat();
    }
}
bool Tracking::RelativePoseEPnp()
{

    // cout << "************come into loop! track reference key frame**************" << endl;
    // 看看描述子是个啥把
    // cout << mCurrentFrame.mDescriptors.rows << ","<< mCurrentFrame.mDescriptors.cols << endl;
    // cout << mLastFrame.mDescriptors.rows << "," << mLastFrame.mDescriptors.cols << endl;
    // Compute Bag of Words vector
    // 步骤1：将当前帧的描述子转化为BoW向量
    if (mCurrentFrame.mbFrameValid)
        mCurrentFrame.ComputeBoW();
    else
        cout << "mCurrentFrame not valid" << endl;
    // 这里也应该计算mLastFrame的BoW吧
    if (mLastFrame.mbFrameValid)
        mLastFrame.ComputeBoW();
    else
        cout << "mLastFrame not valid" << endl;

    // cout << "************come into loop? two frame valid?**************" << endl;
    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    // 步骤2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
    // 特征点的匹配关系由MapPoints进行维护
    // 参数： 第一个是参考帧，第二个是判断的帧
    // 也就是说，第一个是自己，第二个是别人
    // 以后，自己叫做
    // cout << "************come into loop? matcher?**************" << endl;
    // 以currentFrame为基准，然后和后面的lastFrame匹配
    vector<int> rot;
    vector<int> indexs;
    int nmatches = matcher.SearchByBoW(&mLastFrame, mCurrentFrame, vpMapPointMatches, rot,indexs);
    // cout << "************come into loop? search bow?**************" << endl;
    // cout << nmatches << endl;
    for (int i = 0; i < vpMapPointMatches.size(); i++)
    {
    }
    if (nmatches < 15)
    {
        std::cout << "not enough matches" << std::endl;
        return false;
    }
    printf("there are %d matchs feature\n", nmatches);
    // 初始化PnPsolver
    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vpMapPointMatches);
    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
    vector<bool> vbInliers;
    int nInliers;
    bool bNoMore;
    // 通过EPnP算法估计姿态
    cv::Mat Tcw = pSolver->iterate(15, bNoMore, vbInliers, nInliers);
    if (bNoMore)
    {
        printf("Ransac reach max\n");
    }
    bool bMatch = false;
    if (!Tcw.empty())
    {
        Tcw.copyTo(mCurrentFrame.mTcw);

        set<MapPoint *> sFound;

        const int np = vbInliers.size();

        for (int j = 0; j < np; j++)
        {
            if (vbInliers[j])
            {
                mCurrentFrame.mvpMapPoints[j] = vpMapPointMatches[j];
                sFound.insert(vpMapPointMatches[j]);
            }
            else
                mCurrentFrame.mvpMapPoints[j] = NULL;
        }

        // 步骤5：通过PoseOptimization对姿态进行优化求解
        int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
        if (nGood < 10)
        {
            printf("Not enough good matchs via pnp\n");
            return false;
        }

        for (int io = 0; io < mCurrentFrame.N; io++)
            if (mCurrentFrame.mvbOutlier[io])
                mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

        ORBmatcher matcher2(0.9, true);
        if (nGood < 30)
        {
            int nadditional = matcher2.SearchByProjection(mCurrentFrame, mLastFrame, 7, false);

            if (nadditional + nGood >= 50)
            {
                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                // If many inliers but still not enough, search by projection again in a narrower window
                // the camera has been already optimized with many points
                if (nGood > 30 && nGood < 50)
                {
                    sFound.clear();
                    for (int ip = 0; ip < mCurrentFrame.N; ip++)
                        if (mCurrentFrame.mvpMapPoints[ip])
                            sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                    nadditional = matcher2.SearchByProjection(mCurrentFrame, mLastFrame, 3, false);

                    // Final optimization
                    if (nGood + nadditional >= 50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        for (int io = 0; io < mCurrentFrame.N; io++)
                            if (mCurrentFrame.mvbOutlier[io])
                                mCurrentFrame.mvpMapPoints[io] = NULL;
                    }
                }
            }
        }
        if (nGood >= 30)
        {
            bMatch = true;
        }
    }
    else
    {
    }
    //draw_match(vpMapPointMatches);
    // 步骤3:将上一帧的位姿态作为当前帧位姿的初始值
    /*if (debug)
    {
        draw_match(1, rot, vpMapPointMatches);
    }*/

    if (!bMatch)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Tracking::RelativePoseG2o()
{

    // cout << "************come into loop! track reference key frame**************" << endl;
    // 看看描述子是个啥把
    // cout << mCurrentFrame.mDescriptors.rows << ","<< mCurrentFrame.mDescriptors.cols << endl;
    // cout << mLastFrame.mDescriptors.rows << "," << mLastFrame.mDescriptors.cols << endl;
    // Compute Bag of Words vector
    // 步骤1：将当前帧的描述子转化为BoW向量

    // cout << "************come into loop? two frame valid?**************" << endl;
    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    // 步骤2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
    // 特征点的匹配关系由MapPoints进行维护
    // 参数： 第一个是参考帧，第二个是判断的帧
    // 也就是说，第一个是自己，第二个是别人
    // 以后，自己叫做
    // cout << "************come into loop? matcher?**************" << endl;
    // 以currentFrame为基准，然后和后面的lastFrame匹配
    vector<int> rot;
    vector<int> indexs;
    int nmatches = matcher.SearchByBoW(&mLastFrame, mCurrentFrame, vpMapPointMatches, rot,indexs);

    /*cv::Mat img(cv::imread("/home/jena/csq_ws/uav1-1_left.jpg", 0));
    cv::cvtColor(img,img,cv::COLOR_GRAY2BGR);
    for(int i =0;i<indexs.size();i++)
    {
        int r = 10;
        cv::Point2f pt1,pt2;
        pt1.x=mCurrentFrame.mvKeys[indexs[i]].pt.x-r;
        pt1.y=mCurrentFrame.mvKeys[indexs[i]].pt.y-r;
        pt2.x=mCurrentFrame.mvKeys[indexs[i]].pt.x+r;
        pt2.y=mCurrentFrame.mvKeys[indexs[i]].pt.y+r;

        cv::rectangle(img,pt1,pt2,cv::Scalar(0,255,0));
        cv::circle(img,mCurrentFrame.mvKeys[indexs[i]].pt,4,cv::Scalar(0,255,0),-1);
        }
    cv::imwrite("/home/jena/csq_ws/feature_used.jpg",img);*/
    // cout << "************come into loop? search bow?**************" << endl;
    // cout << nmatches << endl;
    // if(nmatches<15)
    //    return false;

    // 步骤3:将上一帧的位姿态作为当前帧位姿的初始值
    /*if (debug)
    {
        draw_match(1, rot, vpMapPointMatches);
    }*/
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw); // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

    // cout << "************come into loop? set pose?**************" << endl;
    // 步骤4:通过优化3D-2D的重投影误差来获得位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // cout << "************come into loop? optimize?**************" << endl;
    // Discard outliers
    // 步骤5：剔除优化后的outlier匹配点（MapPoints）
    int nmatchesMap = 0;
    // cout << "************come into loop**************" << endl;
    // cout << "feature number: "<<mCurrentFrame.N << endl;
    // cout << "mappoint number: "<<mCurrentFrame.mvpMapPoints.size() << endl;
    // cout << "outlier number: "<<mCurrentFrame.mvbOutlier.size() << endl;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                // pMP->mbTrackInView = false;
                // pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if (mCurrentFrame.mvpMapPoints[i]->nObs > 0)
                nmatchesMap++;
        }
    }

    //draw_match(vpMapPointMatches);
    // 在特征点上面画图
    // cout << "**************TrackReferenceKeyFrame********************" << endl;
    // cout << nmatchesMap << endl;
    // traceMap(&mCurrentFrame);
    // cout << "***********TrackReferenceKeyFrame end*******************" << endl;
    // return nmatchesMap>=10;
    return nmatchesMap >= 4;
}

void Tracking::draw_match(vector<MapPoint *> mvpMapPoints)
{
    // mvpMapPoints;
    // for(int i=0;i<mvpMapPoints.size();i++){
    //     if(mvpMapPoints[i]!=NULL){
    //         MapPoint* MP = mvpMapPoints[i];
    //         for(int j=0;j<mLastFrame.mvpMapPoints.size();j++){
    //             if(MP == mLastFrame.mvpMapPoints[j]){
    //                 // 匹配成功
    //                 cv::circle(out_left_1, mCurrentFrame.mvKeysUn[i].pt, 4 * (mCurrentFrame.mvKeysUn[i].octave + 1), cv::Scalar(0, 255, 0), 1);
    //                 cv::circle(out_left_2, mLastFrame.mvKeysUn[j].pt, 4 * (mLastFrame.mvKeysUn[j].octave + 1), cv::Scalar(0, 255, 0), 1);
    //                 cout << "index: " << i <<"," << j << endl;
    //             }
    //         }
    //     }
    // }

    for (int i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
    {
        // 匹配成功
        if (mCurrentFrame.mvpMapPoints[i] != NULL)
        {
            cv::circle(show, mCurrentFrame.mvKeysUn[i].pt, 4 * (mCurrentFrame.mvKeysUn[i].octave + 1), cv::Scalar(0, 255, 0), 1);
        }
    }

    cv::imwrite("/home/jena/csq_ws/debug.jpg", show);
}

void Tracking::traceMap(Frame *frame)
{
    for (auto it = frame->mvpMapPoints.begin();
         it != frame->mvpMapPoints.end(); it++)
    {
        cout << (*it)->mWorldPos << endl;
    }
}

/**
 * @brief 双目和rgbd的地图初始化
 *
 * 由于具有深度信息，直接生成MapPoints
 */
void Tracking::StereoInitialization()
{
    if (mLastFrame.N > 500)
    {
        // Set Frame pose to the origin
        // 步骤1：设定初始位姿
        mLastFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

        // Create KeyFrame
        // 步骤2：将当前帧构造为初始关键帧
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地图3D点、以及BoW
        // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB

        // 问题：需要不需要Map和KeyFrameMap？
        // 特征点的三维空间坐标需要Map的
        // 但是关键帧的Map应该不需要把，就两帧。
        // 所以下面的替换成：
        // KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        mLastFrame.mpMap = mpMap;

        // Insert KeyFrame in the map
        // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
        // 步骤3：在地图中添加该初始关键帧
        mpMap->AddFrame(&mLastFrame);

        // Create MapPoints and asscoiate to KeyFrame
        // 步骤4：为每个特征点构造MapPoint
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                // 步骤4.1：通过反投影得到该特征点的3D坐标
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                // 步骤4.2：将3D点构造为MapPoint
                // 最后一个参数应该是keyframe的ID，这里给个0，不然就是垃圾数据了
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);
                // 步骤4.3：为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围

                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                // NOTE 这个暂时去掉吧啊哈
                pNewMP->AddObservation(&mLastFrame, i);
                // b.从众多观测到该MapPoint的特征点中挑选区分读最高的描述子
                // NOTE 最后也就是更新mDescriptor，因为只有一帧图像，所以删了很多东西
                pNewMP->ComputeDistinctiveDescriptors();
                // c.更新该MapPoint平均观测方向以及观测距离的范围
                // 参考关键帧就是自己，这个方法不知道之后要不要改
                pNewMP->UpdateNormalAndDepth();

                // 步骤4.4：在地图中添加该MapPoint
                mpMap->AddMapPoint(pNewMP);
                // 步骤4.5：表示该KeyFrame的哪个特征点可以观测到哪个3D点
                mLastFrame.AddMapPoint(pNewMP, i);

                // cout << pNewMP->mWorldPos << endl;

                // 步骤4.6：将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mLastFrame.mvpMapPoints[i] = pNewMP;
            }
        }

        cout << "New map created with " << mpMap->mspMapPoints.size() << " points" << endl;

        // 步骤4：在局部地图中添加该初始关键帧
        // mpLocalMapper->InsertKeyFrame(pKFini);

        // mLastFrame = Frame(mCurrentFrame);
        // mnLastKeyFrameId=mCurrentFrame.mnId;
        // mpLastKeyFrame = pKFini;

        // mvpLocalKeyFrames.push_back(pKFini);
        // mvpLocalMapPoints=mpMap->GetAllMapPoints();
        // mpReferenceKF = mLastFrame;
        // mCurrentFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        // ReferenceMapPoints是DrawMapPoints函数画图的时候用的
        // mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        // mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        // mState=OK;
    }
    else
    {
        cout << "mLastFrame.N: " << mLastFrame.N << endl;
    }
}
/*
    * @带初始位姿的初始化
    */
bool Tracking::StereoInitialization(cv::Mat &Tcw)
{
    if (mLastFrame.N > 300)
    {
        // Set Frame pose to the origin
        // 步骤1：设定初始位姿
        mLastFrame.SetPose(Tcw);
        // Create KeyFrame
        // 步骤2：将当前帧构造为初始关键帧
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地图3D点、以及BoW
        // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB

        // 问题：需要不需要Map和KeyFrameMap？
        // 特征点的三维空间坐标需要Map的
        // 但是关键帧的Map应该不需要把，就两帧。
        // 所以下面的替换成：
        // KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        delete mpMap;
        mpMap = new Map();
        mLastFrame.mpMap = mpMap;

        // Insert KeyFrame in the map
        // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
        // 步骤3：在地图中添加该初始关键帧
        mpMap->AddFrame(&mLastFrame);
        // Create MapPoints and asscoiate to KeyFrame
        // 步骤4：为每个特征点构造MapPoint
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                // 步骤4.1：通过反投影得到该特征点的3D坐标
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                // 步骤4.2：将3D点构造为MapPoint
                // 最后一个参数应该是keyframe的ID，这里给个0，不然就是垃圾数据了
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                // 步骤4.3：为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围
                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                // NOTE 这个暂时去掉吧啊哈
                pNewMP->AddObservation(&mLastFrame, i);
                // b.从众多观测到该MapPoint的特征点中挑选区分读最高的描述子
                // NOTE 最后也就是更新mDescriptor，因为只有一帧图像，所以删了很多东西
                pNewMP->ComputeDistinctiveDescriptors();
                // c.更新该MapPoint平均观测方向以及观测距离的范围
                // 参考关键帧就是自己，这个方法不知道之后要不要改
                pNewMP->UpdateNormalAndDepth();
                // 步骤4.4：在地图中添加该MapPoint
                mpMap->AddMapPoint(pNewMP);
                // 步骤4.5：表示该KeyFrame的哪个特征点可以观测到哪个3D点
                // cout << pNewMP->mWorldPos << endl;

                // 步骤4.6：将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mLastFrame.mvpMapPoints[i] = pNewMP;
            }
        }
       
        cout << "New map created with " << mpMap->mspMapPoints.size() << " points" << endl;
        return true;
        // 步骤4：在局部地图中添加该初始关键帧
        // mpLocalMapper->InsertKeyFrame(pKFini);

        // mLastFrame = Frame(mCurrentFrame);
        // mnLastKeyFrameId=mCurrentFrame.mnId;
        // mpLastKeyFrame = pKFini;

        // mvpLocalKeyFrames.push_back(pKFini);
        // mvpLocalMapPoints=mpMap->GetAllMapPoints();
        // mpReferenceKF = mLastFrame;
        // mCurrentFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        // ReferenceMapPoints是DrawMapPoints函数画图的时候用的
        // mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        // mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        // mState=OK;
    }
    else
    {
        cout << "Not enough map point: " << mLastFrame.N << endl;
        return false;
    }
}
bool Tracking::LoopDetecting()
{
    const DBoW2::BowVector &BowVec = mLastFrame.mBowVec;
    const DBoW2::BowVector &CurrentBowVecBowVec = mCurrentFrame.mBowVec;
    double score = mpVocabulary->score(CurrentBowVecBowVec, BowVec);
    std::cout << "test score " << score << std::endl;
    ;
    if (score > 0.055)
        return true;
    return false;
}