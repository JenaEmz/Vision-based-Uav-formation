#ifndef UTILs
#define UTILs
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
using namespace Eigen;
class SlamPoseTrans
{
public:
    static void SE3ToCv(const Eigen::Quaterniond &t_Q, const Eigen::Vector3d &t, cv::Mat &cvMat)
    {
        Matrix3d t_R = t_Q.matrix();

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                cvMat.at<float>(i, j) = t_R(i, j);
            }
        }
        for (int i = 0; i < 3; i++)
        {
            cvMat.at<float>(i, 3) = t(i);
        }
    }

    static void CvToSE3(const cv::Mat &cvMat, Eigen::Quaterniond &t_Q, Eigen::Vector3d &t)
    {
        Eigen::Matrix<double, 3, 3> M;

        M << cvMat.at<float>(0, 0), cvMat.at<float>(0, 1), cvMat.at<float>(0, 2),
            cvMat.at<float>(1, 0), cvMat.at<float>(1, 1), cvMat.at<float>(1, 2),
            cvMat.at<float>(2, 0), cvMat.at<float>(2, 1), cvMat.at<float>(2, 2);
        t_Q = Eigen::Quaterniond(M);
        t << cvMat.at<float>(0, 3), cvMat.at<float>(1, 3), cvMat.at<float>(2, 3);
    }

    static void toCvMatInverse(const cv::Mat &Tcw, cv::Mat &Twc)
    {
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        cv::Mat twc = -Rwc * tcw;
        Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        twc.copyTo(Twc.rowRange(0, 3).col(3));
    }

    //NED中 D已经变成正数
    static void ENUtoNED(const Eigen::Quaterniond &enu_q, const Eigen::Vector3d &enu_t,
                  Eigen::Quaterniond &ned_q, Eigen::Vector3d &ned_t)
    {
        ned_t << enu_t(1), enu_t(0), -enu_t(2);
        ned_q = Eigen::Quaterniond(enu_q.w(), enu_q.y(), enu_q.x(), -enu_q.z());
    }

    static void NEDtoENU(const Eigen::Quaterniond &enu_q, const Eigen::Vector3d &enu_t,
                  Eigen::Quaterniond &ned_q, Eigen::Vector3d &ned_t)
    {
        ned_t << enu_t(1), enu_t(0), -enu_t(2);
        ned_q = Eigen::Quaterniond(enu_q.w(), enu_q.y(), enu_q.x(), -enu_q.z());
    }

    static void LocalposeToSlam(const Eigen::Quaterniond &ned_q, const Eigen::Vector3d &ned_t, cv::Mat &Tcw)
    {
        Eigen::Quaterniond enu_q;
        Eigen::Vector3d enu_t;
        NEDtoENU(ned_q, ned_t, enu_q, enu_t);
        cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);
        SE3ToCv(enu_q, enu_t, Twc);
        toCvMatInverse(Twc, Tcw);
    }
    static void LocalposeToSlam_test(const Eigen::Quaterniond &enu_q, const Eigen::Vector3d &enu_t, cv::Mat &Tcw)
    {
        cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);
        SE3ToCv(enu_q, enu_t, Twc);
        toCvMatInverse(Twc, Tcw);
    }

    static void SlamToLocalpose(const cv::Mat &Tcw, Eigen::Quaterniond &ned_q, Eigen::Vector3d &ned_t)
    {
        /*Eigen::Quaterniond enu_q; Eigen::Vector3d enu_t;
    cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);;
    toCvMatInverse(Tcw,Twc);
    CvToSE3(Twc,enu_q,enu_t);
    ENUtoNED(enu_q,enu_t,ned_q,ned_t);*/

        CvToSE3(Tcw, ned_q, ned_t);
    }
    static void SlamToLocalpose_test(const cv::Mat &Tcw, Eigen::Quaterniond &enu_q, Eigen::Vector3d &enu_t)
    {
        cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);
        Eigen::Vector3d ned_t;
        Eigen::Quaterniond ned_q;

        toCvMatInverse(Tcw, Twc);
        CvToSE3(Twc, ned_q, ned_t);
        enu_t <<  ned_t(2),-ned_t(0), -ned_t(1);
        enu_q = Eigen::Quaterniond(-ned_q.w(), -ned_q.z(), ned_q.x(), ned_q.y());

        //printf("enu_q:%f,%f,%f,%f,ned_t\n",ned_q.w(), ned_q.x(), ned_q.y(), ned_q.z());
        /*cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);*/
        /*enu_q = Eigen::Quaterniond(ned_q.w(), ned_q.x(), ned_q.x(), ned_q.y());
        //printf("new t:%f,%f,%f\n", twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
        printf("enu_q:%f,%f,%f,%f\n",ned_q.w(), ned_q.x(), ned_q.y(), ned_q.z());
        enu_t << ned_t(0), ned_t(2), ned_t(1);*/
    }
};

#endif