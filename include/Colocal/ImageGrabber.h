#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "features.h"

#include "CoLocalSystem.h"

class ImageGrabber
{
  public:
    ImageGrabber(CoLocalSystem *coLocal) : mcoLocal(coLocal) {}
    ImageGrabber(CoLocalSystem *coLocal,const cv::FileStorage& fsSettings);
    void OpencvStereo(cv::Mat &Left, cv::Mat &Right);          //自己的
    cv::Mat OpencvStereo_Other(cv::Mat &Left, cv::Mat &Right); //第二架的
    void RosStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    CoLocalSystem *mcoLocal;
    cv::Mat M1l, M2l, M1r, M2r;
};