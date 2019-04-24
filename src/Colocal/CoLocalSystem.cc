#include "CoLocalSystem.h"

CoLocalSystem::CoLocalSystem(const cv::FileStorage &fsSettings)
{
    fsSettings_ = fsSettings;
    Encoder_init(fsSettings_);
    mTracker = new Tracking(fsSettings_);
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
}

cv::Mat CoLocalSystem::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, int id)
{
    // 这两个完了之后才能做匹配吧
    // 匹配是靠Track这个函数来做的吧
    return mTracker->GrabImageStereo(imLeft, imRight, id);
}
void CoLocalSystem::Encoder_init(const cv::FileStorage &fSettings)
{

    int imgWidth = fSettings["LEFT.width"];
    int imgHeight = fSettings["LEFT.height"];
    ;
    int bufferSize = 1;
    bool inter = true;
    bool stereo = true;
    bool depth = false;
    bool bVocLoad = false; // chose loading method based on file extension
    string dictPath = fSettings["DictPath"];
    bVocLoad = mpVocabulary.loadFromBinaryFile(dictPath);
    string settings_path = fSettings["model_path"];
    codingModel.load(settings_path);
    int nLevels = fSettings["ORBextractor.nLevels"];
    float fx = fSettings["Camera.fx"];
    float bf = fSettings["Camera.bf"];
    float mBaseline = bf / fx;
    float mFocalLength = fx;
    mEncoder = new LBFC2::FeatureCoder(mpVocabulary, codingModel, imgWidth, imgHeight, nLevels, 32, bufferSize, true, stereo, depth, mFocalLength, mBaseline);
}

CoLocalSystem::~CoLocalSystem()
{
    delete mTracker;
}
Tracking *CoLocalSystem::GetTracker()
{
    return mTracker;
}
bool CoLocalSystem::SetCurrentFrame(cv::Mat &imLeft, cv::Mat &imRight, cv::Mat &Tcw)
{
    cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
    return mTracker->SetCurrentFrame(imLeft, imRight);
}
cv::Mat CoLocalSystem::TrackFromImage(cv::Mat &imLeft0, cv::Mat &imRight0,cv::Mat &imLeft1, cv::Mat &imRight1)
{
    
    if (!mTracker->TrackStereo(imLeft0, imRight0))
    {
        std::cout << "Failed to set init frame" << std::endl;
    }
    if (!mTracker->SetCurrentFrame(imLeft1, imRight1))
    {
        std::cout << "Failed to set current frame" << std::endl;
    }
    return mTracker->GetRelPose();
}
cv::Mat CoLocalSystem::TrackFromBitstream(std::vector<uchar> &img_bitstream, cv::Mat &Tcw, const cv::Mat &imLeft, const cv::Mat &imRight)
{
    std::vector<unsigned int> vDecVisualWords;
    std::vector<cv::KeyPoint> vDecKeypointsLeft, vDecKeypointsRight;
    cv::Mat decDescriptorsLeft, decDescriptorsRight;
    
    mEncoder->decodeImageStereo(img_bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);

    if (!mTracker->TrackStereoBitstream(Tcw, vDecKeypointsLeft, decDescriptorsLeft, vDecVisualWords, vDecKeypointsRight, decDescriptorsRight))
    {
        std::cout << "Failed to set last frame" << std::endl;
        return cv::Mat();
    }
    if (!mTracker->SetCurrentFrame(imLeft, imRight))
    {
        std::cout << "Failed to set current frame" << std::endl;
    }
    return mTracker->GetRelPose();
}
void CoLocalSystem::GenerateFeatureBitstream(const cv::Mat &imLeft, const cv::Mat &imRight, std::vector<uchar> &bitstream)
{
    cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
    std::vector<cv::KeyPoint> keypointsLeft, keypointsRight;
    cv::Mat descriptorsLeft, descriptorsRight;

    //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    std::thread threadLeft(&CoLocalSystem::ExtractORB, this, 0, imLeft, std::ref(keypointsLeft), std::ref(descriptorsLeft));
    std::thread threadRight(&CoLocalSystem::ExtractORB, this, 1, imRight, std::ref(keypointsRight), std::ref(descriptorsRight));
    threadLeft.join();
    threadRight.join();
    std::cout<<"First image has "<<keypointsLeft.size()<<"keypoints"<<std::endl;
    mEncoder->encodeImageStereo(keypointsLeft, descriptorsLeft, keypointsRight, descriptorsRight, bitstream);
}

void CoLocalSystem::ExtractORB(int flag, const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys, cv::Mat &descriptors)
{

    if (flag == 0)
    {
        (*mTracker->mpORBextractorLeft)(im, cv::Mat(), vKeys, descriptors);
    }
    else
    {
        (*mTracker->mpORBextractorRight)(im, cv::Mat(), vKeys, descriptors);
    }
}