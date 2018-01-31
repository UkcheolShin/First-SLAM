/*********************************************************************
@ title 1st SLAM Implement 
@ file Tracking.h
@ date 2018-1-29
@ author ukcheol shin
@ brief 입력받은 Frame에 대해서 추적,자세 추정등 을 수행
**********************************************************************/
#ifndef __TRACKING_H__
#define __TRACKING_H__

#include "frame.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/viz.hpp>
#include <fstream>

#define MIN_NUM_FEAT 450
#define KITTI

namespace F_SLAM{

class Frame;      
/// @brief 입력받은 Frame에 대해서 tracking, pose estimation.
class Tracking {
private:
    Frame img_prev, img_curr; /// 이전 이미지, 현재 이미지
    cv::Mat *img_keyframe;    /// 키프레임 리스트를 가르키는 포인터

    cv::Mat inliers;  
    cv::Mat Rot_p2c = cv::Mat::eye(3,3,CV_64F);  /// Rotation : Prev Cam to Curr Cam
    cv::Mat tr_p2c = cv::Mat::zeros(3,1,CV_64F); /// Translation : Prev Cam to Curr Cam
    cv::Mat Rot_w2c = cv::Mat::eye(3,3,CV_64F);  /// Rotation : World to Camera 
    cv::Mat tr_w2c = cv::Mat::zeros(3,1,CV_64F); /// Translation : World to Camera
 
    std::vector<std::vector<cv::Point3f> > objectPoints; /// Triangulate Matched Point 
    std::vector<std::vector<cv::Point2f> > imagePoints;  /// Matched Point between Prev & Curr img

    cv::Mat cameraMatrix; /// Camera Matrix
    cv::Mat cameraDistCoeffs; /// Camera Distortion Matrix

public:
    ///@brief 현재 이미지와 이전 이미지를 입력 받음.
    Tracking( ) : img_prev(cv::Mat()), img_curr(cv::Mat()) 
    { Read_CamParam(); }

    Tracking(cv::Mat& img1)  : img_curr(cv::Mat())
    {
        if (!img1.data)  
            std::cout<< " --(!) Error reading images " << std::endl;
        else
        {        
            img1.copyTo(img_prev.image);
            std::cout <<"file Read Sucess!"<< std::endl;
        }
        Read_CamParam();
    }

    Tracking(cv::Mat& img1, cv::Mat& img2) 
    {
        if (!img1.data)  
            std::cout<< " --(!) Error reading images " << std::endl;
        else
        {
            img1.copyTo(img_prev.image);
            img2.copyTo(img_curr.image);
            std::cout <<"file Read Sucess!"<< std::endl;
        }
        Read_CamParam();
    }

    /// @brief Initialze Visual Odometry Class Module
    void Initialize_Tracking()
    {
        Tracking::featureDetection(img_prev);
//        Tracking::featureTracking();
    }

    /// @brief 입력받은 이미지에 대해서 특징점 검출
    void featureDetection(Frame &img)
    {
        img.DetectKeypoint(1);
        std::cout << "Num of Detected Keypoint : "<< img.keypoints.size() <<std::endl <<std::endl; 
    }    

    Frame GrabImage(const cv::Mat &im, const double &timestamp)
    {
      // image을 받아와 tracking class의 member함수에 저장
      mImGray = im;
      if(mImGray.channels() == 3)
        cvtColor(mImGray,mImGray,CV_BGR2GRAY)
      // Frame object 생성 
      return Frame(mImGray,timestamp,mK,mDistCoef);

    }

    /// @brief 이전 프레임의 특징점을 기반으로 Optical Flow tracking.
    void featureTracking()
    {   
        std::vector<uchar> status;
        std::vector<float> err;                    
        cv::Size winSize=cv::Size(21,21);                                                                                             
        cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

        //img1 ~ img2 간의 optical flow 계산
        cv::calcOpticalFlowPyrLK(img_prev.image, img_curr.image, 
                                img_prev.keypoints, img_curr.keypoints, 
                                status, err, winSize, 3, termcrit, 0, 0.001);

        // KLT tracking 실패한 점을 제거.
        int indexCorrection = 0;
        for( int i=0; i<status.size(); i++)
        {  
            cv::Point2f pt = img_curr.keypoints.at(i- indexCorrection);
            if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))    {
                  if((pt.x<0)||(pt.y<0))    {
                    status.at(i) = 0;
                  }
                  img_prev.keypoints.erase (img_prev.keypoints.begin() + (i - indexCorrection));
                  img_curr.keypoints.erase (img_curr.keypoints.begin() + (i - indexCorrection));
                  indexCorrection++;
              }
        }
    }

    /// @brief 현재 프레임을 기준으로 tracking된 결과를 visualize하여 output Matrix에 저장
    void handleTrackedPoints(cv::Mat &output) 
    {
        //cv::cvtColor(frame, gray, CV_BGR2GRAY); 
        img_curr.image.copyTo(output);

        //모든 traked된 특징점에 대해서 선과 원을 그림
        for(int i= 0; i < img_curr.keypoints.size(); i++ ) 
        {
          cv::line(output, img_prev.keypoints[i], img_curr.keypoints[i], cv::Scalar(255,255,255));
          cv::circle(output, img_curr.keypoints[i], 2, cv::Scalar(255,255,255),-1);
          cv::circle(output, img_prev.keypoints[i], 2, cv::Scalar(255,255,255),-1);
        }
    }

    /// @brief 매칭쌍으로부터 E를 구하고 R,t로 분해 
    void calcPose(cv::Mat &output_inlier)
    {  
        double focal = 718.8560;          // Kitti Calibration file의 데이터 사용. calib file 참고.
        cv::Point2d pp(607.1928, 185.2157);    // WARNING: different sequences in the KITTI Tracking dataset have different intrinsic/extrinsic parameters

        cv::Mat E = cv::findEssentialMat(img_curr.keypoints, img_prev.keypoints, focal, pp, cv::RANSAC, 0.999, 1.0, inliers);
        cv::recoverPose(E, img_curr.keypoints, img_prev.keypoints, Rot_p2c, tr_p2c, focal, pp, inliers);

        int numberOfPts = cv::sum(inliers)[0];
    //    std::cout << "Number of inliers: " << numberOfPts << std::endl;  
    //    std::cout << "rotation:" << Rot_p2c << std::endl;
    //    std::cout << "translation:" << tr_p2c << std::endl;

        inliers.copyTo(output_inlier);
    }

    /// @brief R,t를 받아 World Based coordinate로부터의 현재 6DOF를 구함. 
    void w2c_pose(double scale, cv::Mat &R, cv::Mat &t)
    {
        tr_w2c += scale*(Rot_w2c*tr_p2c);
        Rot_w2c *= Rot_p2c;

        Rot_w2c.copyTo(R);
        tr_w2c.copyTo(t);
    }

    /// @brief 새로운 이미지가 들어올시 이전 프레임과 현재 프레임을 이동
    void update()
    {
        img_prev.keypoints.clear();
        img_prev.keypoints = img_curr.keypoints;
        img_prev.image = img_curr.image.clone();
        img_curr.keypoints.clear();
        
//        img_prev.image = img_curr.image;
//        img_prev.keypoints = img_curr.keypoints;
    }

    /// @brief 추적하는 KeyPoint수가 적어지면, 재 추정
    void checkKeyPoint()
    {
      if (img_prev.keypoints.size() < MIN_NUM_FEAT)    
      {
        std::cout << "Number of tracked features reduced to " << img_prev.keypoints.size() << std::endl;
        std::cout << "trigerring redection" << std::endl;

        featureDetection(img_prev);
        featureTracking();
      }
    }

    /// @brief Matched Point를 triangulate
    void triangulation(cv::Mat &R, cv::Mat &t, std::vector<cv::Vec3d> &points3D )
    {
      // R,t로부터 projection matrix 구성.
      // 한쪽을 [I|0]이라 생각, 다른 한쪽을 [R|t]라 생각.
      cv::Mat projection2(3, 4, CV_64F);        // the 3x4 projection matrix
      R.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
      t.copyTo(projection2.colRange(3, 4));

      // compose generic projection matrix 
      cv::Mat projection1(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
      cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
      diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));

//      std::cout << "First Projection matrix=" << projection1 << std::endl;
//      std::cout << "Second Projection matrix=" << projection2 << std::endl;

      // inlier point 들을 구성하여 undistort하고, triangulate함.
      // to contain the inliers
      std::vector<cv::Vec2d> inlierPts1;
      std::vector<cv::Vec2d> inlierPts2;

      // create inliers input point vector for triangulation
      int j(0); 
      for (int i = 0; i < inliers.rows; i++) {

        if (inliers.at<uchar>(i)) {
          inlierPts1.push_back(cv::Vec2d(img_prev.keypoints[i].x, img_prev.keypoints[i].y));
          inlierPts2.push_back(cv::Vec2d(img_curr.keypoints[i].x, img_curr.keypoints[i].y));
        }
      }

      // undistort and normalize the image points
      std::vector<cv::Vec2d> points1u;
      cv::undistortPoints(inlierPts1, points1u, cameraMatrix, cameraDistCoeffs);
      std::vector<cv::Vec2d> points2u;
      cv::undistortPoints(inlierPts2, points2u, cameraMatrix, cameraDistCoeffs);

     // triangulation
      triangulate(projection1, projection2, inlierPts1, inlierPts2, points3D);

    }

    /**
    @ brief triangulate using Liner LS-Method
    */
    cv::Vec3d triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2) 
    {
      // system of equations assuming image=[u,v] and X=[x,y,z,1]
      // from u(p3.X)= p1.X and v(p3.X)=p2.X
      cv::Matx43d A(u1(0)*p1.at<double>(2, 0) - p1.at<double>(0, 0), u1(0)*p1.at<double>(2, 1) - p1.at<double>(0, 1), u1(0)*p1.at<double>(2, 2) - p1.at<double>(0, 2),
        u1(1)*p1.at<double>(2, 0) - p1.at<double>(1, 0), u1(1)*p1.at<double>(2, 1) - p1.at<double>(1, 1), u1(1)*p1.at<double>(2, 2) - p1.at<double>(1, 2),
        u2(0)*p2.at<double>(2, 0) - p2.at<double>(0, 0), u2(0)*p2.at<double>(2, 1) - p2.at<double>(0, 1), u2(0)*p2.at<double>(2, 2) - p2.at<double>(0, 2),
        u2(1)*p2.at<double>(2, 0) - p2.at<double>(1, 0), u2(1)*p2.at<double>(2, 1) - p2.at<double>(1, 1), u2(1)*p2.at<double>(2, 2) - p2.at<double>(1, 2));

      cv::Matx41d B(p1.at<double>(0, 3) - u1(0)*p1.at<double>(2, 3),
                  p1.at<double>(1, 3) - u1(1)*p1.at<double>(2, 3),
                  p2.at<double>(0, 3) - u2(0)*p2.at<double>(2, 3),
                  p2.at<double>(1, 3) - u2(1)*p2.at<double>(2, 3));

      // X contains the 3D coordinate of the reconstructed point
      cv::Vec3d X;

      // solve AX=B
      cv::solve(A, B, X, cv::DECOMP_SVD);

      return X;
    }

    /**
    @ brief triangulate a vector of image points
    */
    void triangulate(const cv::Mat &p1, const cv::Mat &p2, const std::vector<cv::Vec2d> &pts1, const std::vector<cv::Vec2d> &pts2, std::vector<cv::Vec3d> &pts3D) {

      for (int i = 0; i < pts1.size(); i++) {

        pts3D.push_back(triangulate(p1, p2, pts1[i], pts2[i]));
      }
    }

    void Read_CamParam()
    {
      // 카메라 보정 파라미터 읽어옴  
      // Read the camera calibration parameters
      cv::FileStorage fs("calib.xml", cv::FileStorage::READ);
      fs["Intrinsic"] >> cameraMatrix;
      fs["Distortion"] >> cameraDistCoeffs;

//      cameraMatrix.at<double>(0, 2) = 268.;
//      cameraMatrix.at<double>(1, 2) = 178;

      std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
      std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
      std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
      std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl << std::endl;
      cv::Matx33f cMatrix(cameraMatrix);

    }

    /// @brief 새로운 입력 프레임을 지정
    int SetCurImage(cv::Mat& img)
    {
        if (!img.data)  
        {
            std::cout<< " --(!) Error reading images " << std::endl;
            return -1;
        }
        else
        {        
            img.copyTo(img_curr.image);
            std::cout <<"file Read Sucess!"<< std::endl;
            return 0;
        }
    }


#ifdef KITTI
    double getAbsoluteScale(int frame_id, cv::Point3d &xyz)  
    {  
      std::string line;
      int i = 0;
      std::ifstream myfile ("/media/sf_VirtualBox_share0/data_odometry_poses/dataset/poses/00.txt");
      double x =0, y=0, z = 0;
      double x_prev, y_prev, z_prev;
      if (myfile.is_open())
      {
        while (( std::getline (myfile,line) ) && (i<=frame_id))
        {
          x_prev = x; y_prev = y; z_prev = z; 
          std::istringstream in(line);
          //cout << line << '\n';
          for (int j=0; j<12; j++)  {
            in >> z ;
            if (j==7) y=z;
            if (j==3)  x=z;
          }
          xyz.x = x;  xyz.y = y; xyz.z = z;
          i++;
        }
        myfile.close();
      }

      else {
        std::cout << "Unable to open file";
        return 0;
      }

      return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
    }
#endif

    /// @brief Select Keyframe 
    void KeyFrameSelection(void)
    {
      /*
       object : km+1을 선택하자 
       1. It-1, It 연속된 2개의 이미지.
       2. Km : last selected keyframe 0<= m < t-1
       기준
       1.It ~ Km간의 Rotation을 구해서 그게 15도 이상이면 안됨. 
        너무 많이 움직인것임.
         It-1 을 Km+1로
       2.It ~ Km 간의 Matching 상의 결과 비교
        Km 과 It에서 공통되게 검출된 Matching(matching 정합 결과)  /  Km의 특징점 개수  < 평균 공통 개수 비율* 0.3 이면 Not reliable
        이럴 경우에는 scene, illumination change가 있는 경우이므로
        It-1 을 Km+1로
       3.3D-2D 매칭쌍이 <250개면 It-1을 Km+1로
       4.It ~ Km의 매칭쌍의 픽셀간 평균 거리 차이가 >image width * 0.2면 충분히 많이 움직였으므로
          It 를 Km으로. 
       5. 위의 조건중 아무것도 충족이 안될 경우에는 시간이 1초 지났을 경우의 It를 Km+1로 함  
      */  
    }
/*
bool Tracking::NeedNewKeyFrame()
{
    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}*/

}; // class

} // namespace

#endif