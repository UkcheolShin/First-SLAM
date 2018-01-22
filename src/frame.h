/*********************************************************************
@ title 1st SLAM Implement 
@ file Freme.h
@ date 2018-01-22
@ brief Frame에 대한 header 파일
@ what_i_do member 함수 추가.
**********************************************************************/
#ifndef FRAME_H
#define FRAME_H


#include "header.h"


namespace F_SLAM{
/**
@brief Frame에 대한 처리.
@todo 1.timeStamp의 용도는 무엇인가요?
      
*/

class Frame
{
public:
    Frame(const cv::Mat &imGray, const double &timeStamp,cv::Mat &K,cv::Mat &distCoeff);
 
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

    void extractFAST(cv::Mat img_1, vector<cv::Point2f>& points1);

    /// @brief 이미지에 대해서 키포인트를 검출.
    void DetectKeypoint(const int flag, const cv::Mat &image)
    {   
        cv::Ptr<cv::Feature2D> feature;
        std::vector<cv::KeyPoint> keypoints; /// 입력된 이미지에 대해 검출된 키포인트
    
        switch(flag)
        {
            case 0:
                cv::FAST(image,keypoints, 20, true); // image , keypoints, threshold, nonmaxsuppression
                break;
            case 1:
                feature = cv::ORB::create(1000);
                feature->detect(image, keypoints, cv::noArray());
                break;
            case 2:
                feature = cv::xfeatures2d::SIFT::create(500);  
                feature->detect(image, keypoints, cv::noArray());
                break;
            default:
                std::cout << "input argument error!!" <<std::endl <<std::endl;    
        }
        cv::KeyPoint::convert(keypoints, keypoints_f, std::vector<int>());
    }

    ~Frame() { }

public:
    //
    cv::Mat image;  /// 입력된 이미지
    std::vector<cv::Point2f> keypoints_f; /// 키포인트.

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

};

bool Frame::mbInitialComputations=true;
}

#endif // FRAME_H