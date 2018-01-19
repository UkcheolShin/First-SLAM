#ifndef FRAME_H
#define FRAME_H

#endif // FRAME_H

#include "header.h"


namespace kbSLAM{

class frame
{
public:
    frame(const cv::Mat &imGray, const double &timeStamp,cv::Mat &K,cv::Mat &distCoeff);
    void extractFAST(cv::Mat img_1, vector<cv::Point2f>& points1);
    vector<cv::KeyPoint> m_keypoints;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

};

}
