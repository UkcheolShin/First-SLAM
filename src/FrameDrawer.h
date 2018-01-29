/*
@ brief 1st SLAM Implement 
@ file FrameDrawer.h
@ date 2018-1-28
@ author ukcheol shin
@ brief header 파일
*/
#ifndef __FRAMEDRAWER_H__
#define __FRAMEDRAWER_H__

#include "frame.h"
#include "header.h"

using namespace cv;
using namespace std;

namespace F_SLAM{
class Frame;
class Tracking;
class FrameDrawer{
  public : 
    void odometry_draw2D(cv::Mat t_f, cv::Point3d xyz, cv::Mat &traj)
    {
      int fontFace = FONT_HERSHEY_PLAIN;
      double fontScale = 1;
      int thickness = 1;  
      int offset_x = 300, offset_z = 100;
      char text[100];

      cv::Point textOrg(10, 30);
      cv::Point textOrg2(10, 50);

      int x = int(t_f.at<double>(0)) + offset_x;
      int z = int(t_f.at<double>(2)) + offset_z;

    //  traj = Mat::zeros(600, 600, CV_8UC3);
      cv::circle(traj, cv::Point(offset_x+xyz.x, offset_z+xyz.z) ,1, CV_RGB(0,255,0), 2);
      cv::circle(traj, cv::Point(x, z) ,1, CV_RGB(255,0,0), 2);

      cv::rectangle(traj, cv::Point(10, 10), cv::Point(600, 50), CV_RGB(0,0,0), CV_FILLED);
      cv::rectangle(traj, cv::Point(10, 30), cv::Point(600, 50), CV_RGB(0,0,0), CV_FILLED);

      sprintf(text, "Coordinates(Our): x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
      cv::putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

      sprintf(text, "Coordinates(GT ): x = %02fm y = %02fm z = %02fm", xyz.x, xyz.y, xyz.z);
      cv::putText(traj, text, textOrg2, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

    }
}; //end class 
} //end namespace

#endif