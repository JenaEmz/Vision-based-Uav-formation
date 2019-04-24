#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"

#include "string"

float Frame::cx, Frame::cy;
float Frame::fx, Frame::fy;
float Frame::invfx, Frame::invfy;
bool Frame::mbInitialComputations = true;
float Frame::mnMinX, Frame::mnMaxX, Frame::mnMinY, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame(const cv::Mat &imLeft,
             const cv::Mat &imRight,
             ORBextractor *extractorLeft,
             ORBextractor *extractorRight, cv::Mat &K, cv::Mat &dist, ORBVocabulary *voc, float baseline, int craft_id,ImageInfo Info) : mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight),
                                                                                                                          mCraftID(craft_id), mbFrameValid(false), mpORBvocabulary(voc),
                                                                                                                          mbf(baseline), mK(K.clone()), mDistCoef(dist.clone())
{
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    //mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
    thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
    thread threadRight(&Frame::ExtractORB, this, 1, imRight);
    threadLeft.join();
    threadRight.join();
    //debug
    /*if(1){
        cv::Mat img, imgRight;
        imLeft.copyTo(img);
        imRight.copyTo(imgRight);
        cv::cvtColor(img,img,cv::COLOR_GRAY2BGR);
        cv::cvtColor(imgRight,imgRight,cv::COLOR_GRAY2BGR);
        std::for_each(mvKeys.begin(), mvKeys.end(), [&](cv::KeyPoint i) {
            cv::circle(img, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
        });
        std::for_each(mvKeysRight.begin(), mvKeysRight.end(), [&](cv::KeyPoint i) {
            cv::circle(imgRight, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
        });
        // cv::imshow("left extractor", img);
        // cv::imshow("right extractor", imgRight);
        // cv::waitKey(0);

        cv::imwrite("/home/zbf/h"+std::to_string(craft_id)+".png",img);
        cv::imwrite("/home/zbf/h"+std::to_string(craft_id)+"2.png",imgRight);
    }*/

    //debug end
    N = mvKeys.size();

    if (mvKeys.empty())
        return;

    // UndistortKeyPoints();
    mvKeysUn = mvKeys;
    mvuRight = vector<float>(N, -1);
    mvDepth = vector<float>(N, -1);

    mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
    mvbOutlier = vector<bool>(N, false);
    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
        mnMinX = 0.0f;
        mnMaxX = Info.cols;
        mnMinY = 0.0f;
        mnMaxY = Info.rows;

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    AssignFeaturesToGrid();

    ComputeStereoMatches();
}

/*
* @brief 根据压缩-解压后的特征创建帧
*/
Frame::Frame(const std::vector<cv::KeyPoint> &keyPointsLeft, const cv::Mat &descriptorLeft,
             const std::vector<unsigned int> &visualWords, const std::vector<cv::KeyPoint> &keyPointsRight,
             const cv::Mat &descriptorRight, ORBextractor *extractorLeft,
             ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &dist, const float &baseline,
             const float &thDepth,ImageInfo Info)
    : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), 
      mK(K.clone()), mDistCoef(dist.clone()), mbf(baseline)
{
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    //mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    mvKeys = keyPointsLeft;
	mvKeysRight = keyPointsRight;
	mDescriptors = descriptorLeft;
    mDescriptorsRight = descriptorRight;

    mvKeysUn = mvKeys;
    N = mvKeys.size();
    std::cout<<"Frame has "<<N<<" left keypoint"<<std::endl;
    
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
	if(mvKeys.empty())
		return;
    mBowVec.clear();
	mFeatVec.clear();
	// Convert visual words
	/*mBowVec.clear();
	mFeatVec.clear();
     std::cout<<"1";
	for( unsigned int i_feature = 0; i_feature < visualWords.size(); i_feature++ )
	{
		const unsigned int &wordId = visualWords[i_feature];
		DBoW2::WordValue v = mpORBvocabulary->getWordWeight(wordId);
		DBoW2::NodeId nid = mpORBvocabulary->getParentNode(wordId, 4);

		mBowVec.addWeight(wordId, v);
		mFeatVec.addFeature(nid, i_feature);
	}*/
    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
        mnMinX = 0.0f;
        mnMaxX = Info.cols;
        mnMinY = 0.0f;
        mnMaxY = Info.rows;

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }
    mb = mbf / fx;
    AssignFeaturesToGrid();
    ComputeStereoMatches();
}
    void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if (flag == 0)
        (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);
    else
        (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
}

/**
 * @brief 双目匹配
 *
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
    if (nMinCellX >= FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
    if (nMinCellY >= FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
    if (nMaxCellY < 0)
        return vIndices;

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if (vCell.empty())
                continue;

            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if (bCheckLevels)
                {
                    if (kpUn.octave < minLevel)
                        continue;
                    if (maxLevel >= 0)
                        if (kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

void Frame::ComputeStereoMatches()
{
    // cout << "******************here come in?************************" << endl;
    mvuRight = vector<float>(N, -1.0f);
    mvDepth = vector<float>(N, -1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    // 步骤1：建立特征点搜索范围对应表，一个特征点在一个带状区域内搜索匹配特征点
    // 匹配搜索的时候，不仅仅是在一条横线上搜索，而是在一条横向搜索带上搜索,简而言之，原本每个特征点的纵坐标为1，这里把特征点体积放大，纵坐标占好几行
    // 例如左目图像某个特征点的纵坐标为20，那么在右侧图像上搜索时是在纵坐标为18到22这条带上搜索，搜索带宽度为正负2，搜索带的宽度和特征点所在金字塔层数有关
    // 简单来说，如果纵坐标是20，特征点在图像第20行，那么认为18 19 20 21 22行都有这个特征点
    // vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]都有这个特征点编号
    vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());
    // cout << "Frame.cc vRowIndices size: " << vRowIndices.size() << endl;
    for (int i = 0; i < nRows; i++)
        vRowIndices[i].reserve(200);

    // cout << "Frame.cc vRowIndices item size: " << vRowIndices[0].size() << endl;
    const int Nr = mvKeysRight.size();
    for (int iR = 0; iR < Nr; iR++)
    {
        // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大

        const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY + r);
        const int minr = floor(kpY - r);
        for (int yi = minr; yi <= maxr; yi++)
            vRowIndices[yi].push_back(iR);
    }

    // for(int ii=0;ii<vRowIndices.size();ii++){
    //     cout << vRowIndices[ii].size() << ",";
    // }
    // cout << endl;
    // Set limits for search
    const float minZ = mbf / fx; // NOTE bug mb没有初始化，mb的赋值在构造函数中放在ComputeStereoMatches函数的后面
    const float minD = 0;        // 最小视差, 设置为0即可
    // const float maxD = mbf/minZ;  // 最大视差, 对应最小深度 mbf/minZ = mbf/mb = mbf/(mbf/fx) = fx (wubo???)
    const float maxD = 600; // 最大视差, 对应最小深度 mbf/minZ = mbf/mb = mbf/(mbf/fx) = fx (wubo???)

    // For each left keypoint search a match in the right image
    vector<pair<int, int>> vDistIdx;
    vDistIdx.reserve(N);
    // 步骤2：对左目相机每个特征点，通过描述子在右目带状搜索区域找到匹配点, 再通过SAD做亚像素匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致
    // 这里是不是应该对校正后特征点求深度呢？(wubo???)
    for (int iL = 0; iL < N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;
        // 到点的坐标都没啥问题

        // 可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if (vCandidates.empty())
            continue;

        const float minU = uL - maxD; // 最小匹配范围
        const float maxU = uL - minD; // 最大匹配范围

        if (maxU < 0)
            continue;

        // maxU也能找到

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        // 每个特征点描述子占一行，建立一个指针指向iL特征点对应的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        for (size_t iC = 0; iC < vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            // 仅对近邻尺度的特征点进行匹配
            if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                continue;

            // ANCHOR 这儿通,kpR都是对的
            const float &uR = kpR.pt.x;

            // cout << "kpR.pt.x: "<< kpR.pt.x << " kpR.pt.y: "<< kpR.pt.y << endl;
            if (uR >= minU && uR <= maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
        // 最好的匹配的匹配误差存在bestDist，匹配点位置存在bestIdxR中
        // 这里不对,bestDist 和bestIDxR啥的
        // 通过改这个范围，这个有值了
        // cout << "bestDist: " <<bestDist<<"bestIDxR: "<<bestIdxR<< endl;

        // Subpixel match by correlation
        // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
        if (bestDist < thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x * scaleFactor);
            const float scaledvL = round(kpL.pt.y * scaleFactor);
            const float scaleduR0 = round(uR0 * scaleFactor);

            // sliding window search
            const int w = 5; // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
            IL.convertTo(IL, CV_32F);
            IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F); //窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2 * L + 1); // 11

            // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0 + L - w; //这个地方是否应该是scaleduR0-L-w (wubo???)
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for (int incR = -L; incR <= +L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                IR.convertTo(IR, CV_32F);
                IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F); //窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

                float dist = cv::norm(IL, IR, cv::NORM_L1); // 一范数，计算差的绝对值
                if (dist < bestDist)
                {
                    bestDist = dist; // SAD匹配目前最小匹配偏差
                    bestincR = incR; // SAD匹配目前最佳的修正量
                }

                vDists[L + incR] = dist; // 正常情况下，这里面的数据应该以抛物线形式变化
            }

            if (bestincR == -L || bestincR == L) // 整个滑动窗口过程中，SAD最小值不是以抛物线形式出现，SAD匹配失败，同时放弃求该特征点的深度
                continue;

            // Sub-pixel match (Parabola fitting)
            // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if (deltaR < -1 || deltaR > 1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

            // 这里是disparity，根据它算出depth
            float disparity = (uL - bestuR);

            if (disparity >= minD && disparity < maxD) // 最后判断视差是否在范围内
            {
                if (disparity <= 0)
                {
                    disparity = 0.01;
                    bestuR = uL - 0.01;
                }
                // depth 是在这里计算的
                // depth=baseline*fx/disparity
                mvDepth[iL] = mbf / disparity;                    // 深度
                mvuRight[iL] = bestuR;                            // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int, int>(bestDist, iL)); // 该特征点SAD匹配最小匹配偏差
            }
        }
    }
    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(), vDistIdx.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size() / 2].first;
    const float thDist = 1.5f * 1.4f * median; // 计算自适应距离, 大于此距离的匹配对将剔除

    for (int i = vDistIdx.size() - 1; i >= 0; i--)
    {
        if (vDistIdx[i].first < thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second] = -1;
            mvDepth[vDistIdx[i].second] = -1;
        }
    }
    // trace(mvDepth);
}

std::vector<MapPoint *> Frame::GetMapPointMatches()
{
    return mvpMapPoints;
}

/**
 * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
 * @param  i 第i个keypoint
 * @return   3D点（相对于世界坐标系）
 */
cv::Mat Frame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if (z > 0)
    {
        // 由2维图像反投影到相机坐标系
        // mvDepth是在ComputeStereoMatches函数中求取的
        // mvDepth对应的校正前的特征点，因此这里对校正前特征点反投影
        // 可在Frame::UnprojectStereo中却是对校正后的特征点mvKeysUn反投影
        // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u - cx) * z * invfx;
        const float y = (v - cy) * z * invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
        // unique_lock<mutex> lock(mMutexPose);
        // 由相机坐标系转换到世界坐标系
        // Twc为相机坐标系到世界坐标系的变换矩阵
        // Twc.rosRange(0,3).colRange(0,3)取Twc矩阵的前3行与前3列
        return mTcw.rowRange(0, 3).colRange(0, 3) * x3Dc + mTcw.rowRange(0, 3).col(3);
    }
    else
        return cv::Mat();
}

void Frame::trace(std::vector<float> vec)
{
    for (auto it = vec.begin(); it != vec.end(); it++)
    {
        cout << *it << ",";
    }
    cout << endl;
}
/**
 * @brief Add MapPoint to KeyFrame
 * @param pMP MapPoint
 * @param idx MapPoint在KeyFrame中的索引
 */
void Frame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    // unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx] = pMP;
}

/**
 * @brief Bag of Words Representation
 *
 * 计算词包mBowVec和mFeatVec，其中mFeatVec记录了属于第i个node（在第4层）的ni个描述子
 * @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
 */
void Frame::ComputeBoW()
{
    if (mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // cout << "vCurrentDesc size: "<< vCurrentDesc.size() << endl;
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        // cout << "mBowVec size: "<<mBowVec.size() << endl;
        // cout << "mFeatVec size: "<<mFeatVec.size() << endl;
    }
}
Frame::Frame()
{
}

/**
 * @brief Set the camera pose.
 * 
 * 设置相机姿态，随后会调用 UpdatePoseMatrices() 来改变mRcw,mRwc等变量的值
 * @param Tcw Transformation from world to camera
 */
void Frame::SetPose(cv::Mat Tcw)
{
    // KeyFrame的

    // Tcw.copyTo(mTcw);
    // cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    // cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    // cv::Mat Rwc = Rcw.t();
    // Ow = -Rwc*tcw;

    // Twc = cv::Mat::eye(4,4,Tcw.type());
    // Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    // Ow.copyTo(Twc.rowRange(0,3).col(3));
    // // center为相机坐标系（左目）下，立体相机中心的坐标
    // // 立体相机中心点坐标与左目相机坐标之间只是在x轴上相差mHalfBaseline,
    // // 因此可以看出，立体相机中两个摄像头的连线为x轴，正方向为左目相机指向右目相机
    // cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    // // 世界坐标系下，左目相机中心到立体相机中心的向量，方向由左目相机指向立体相机中心
    // Cw = Twc*center;

    // Frame的
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

/**
 * @brief Computes rotation, translation and camera center matrices from the camera pose.
 *
 * 根据Tcw计算mRcw、mtcw和mRwc、mOw
 */
void Frame::UpdatePoseMatrices()
{
    // [x_camera 1] = [R|t]*[x_world 1]，坐标为齐次形式
    // x_camera = R*x_world + t
    mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0, 3).col(3);
    // mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
    // mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系
    mOw = -mRcw.t() * mtcw;
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    /*if (mDistCoef.at<float>(0) != 0.0)
    {
        // 矫正前四个边界点：(0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4, 2, CV_32F);
        mat.at<float>(0, 0) = 0.0; //左上
        mat.at<float>(0, 1) = 0.0;
        mat.at<float>(1, 0) = imLeft.cols; //右上
        mat.at<float>(1, 1) = 0.0;
        mat.at<float>(2, 0) = 0.0; //左下
        mat.at<float>(2, 1) = imLeft.rows;
        mat.at<float>(3, 0) = imLeft.cols; //右下
        mat.at<float>(3, 1) = imLeft.rows;

        // Undistort corners
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0)); //左上和左下横坐标最小的
        mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0)); //右上和右下横坐标最大的
        mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1)); //左上和右上纵坐标最小的
        mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1)); //左下和右下纵坐标最小的
    }
    else
    {*/
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    //}
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
            mGrid[i][j].reserve(nReserve);

    // 在mGrid中记录了各特征点
    for (int i = 0; i < N; i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if (PosInGrid(kp, nGridPosX, nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
    posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
        return false;

    return true;
}
