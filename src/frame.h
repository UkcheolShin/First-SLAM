/*********************************************************************
@ title 1st SLAM Implement 
@ file Freme.h
@ date 2018-01-22
@ brief Frame에 대한 header 파일
@ what_i_do member 함수 추가.
**********************************************************************/
#ifndef __FRAME_H__
#define __FRAME_H__

#include "header.h"

namespace F_SLAM{

//bool Frame::mbInitialComputations=true;
/**
@brief Frame에 대한 정보 처리.
*/
class Frame{
private:

public:
    cv::Mat image;  /// 입력된 이미지
    std::vector<cv::Point2f> keypoints; /// 접근 가능한 키포인트.

    // Current and Next Frame id
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    //KeyFrame* mpReferenceKF;

    // Rotation, Translation and camera center
    cv::Mat mR2W;
    cv::Mat mTcW;

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
    
    //boolean flag
    static bool mbInitialComputations;

    Frame() : image(cv::Mat())
    {}

    /// @brief 입력된 이미지에 대해서 GRAY 스케일로 변환하여 저장.
    Frame(const cv::Mat &input_image) : image(cv::Mat())
    {
        image = input_image;
        if(image.channels() == 3)
            cv::cvtColor(image, image, CV_BGR2GRAY); 
    } 

    /// @brief 입력된 이미지 경로에 대해서 읽어 온 후 GRAY 스케일로 변환하여 저장.
    Frame(const char *filename) : image(cv::Mat())
    {
        image = cv::imread(filename);
        if(image.channels() == 3)
            cv::cvtColor(image, image, CV_BGR2GRAY); 
    }
 /*
    /// @brief 입력된 이미지에 대해서 GRAY 스케일로 변환하여 저장.
    Frame(const cv::Mat &imGray, const double &timeStamp, cv::Mat &K, cv::Mat &distCoef)
        :mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()),
    {
        // rgb --> gray
        if(imGray.channels()==3)
            cvtColor(imGray,imGray,CV_RGB2GRAY);
            
        // ORB extraction
        DetectKeypoint(0,imGray);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }
    }
*/
    /// @brief 이미지에 대해서 키포인트를 검출.
    void DetectKeypoint(const int flag)
    {   
        cv::Ptr<cv::Feature2D> feature;
        std::vector<cv::KeyPoint> tmp_keypoints; /// 입력된 이미지에 대해 검출된 키포인트
        switch(flag)
        {
            case 0:
                cv::FAST(image,tmp_keypoints, 20, true); // image , keypoints, threshold, nonmaxsuppression
                break;
            case 1:
                feature = cv::ORB::create(1000);
                feature->detect(image, tmp_keypoints, cv::noArray());
                break;
            case 2:
                feature = cv::xfeatures2d::SIFT::create(500);  
                feature->detect(image, tmp_keypoints, cv::noArray());
                break;
            default:
                std::cout << "input argument error!!" <<std::endl <<std::endl;    
        }
        cv::KeyPoint::convert(tmp_keypoints, keypoints, std::vector<int>());
    }
/* to do list
    /// @brief Distorted image --> Undistorted image
    cv::Mat remap(const cv::Mat &image, cv::Size &outputSize); 

    /// @brief read Camera Matrix file
    cv::Mat getCameraMatrix() { return cameraMatrix; }

    /// @brief read Camera Distortion Matrix file
    cv::Mat getDistCoeffs()   { return distCoeffs; }
*/
    ~Frame() { }
};

} // namespace
#endif