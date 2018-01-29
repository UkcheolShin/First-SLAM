/*********************************************************************
@ title 1st parital SLAM Implement 
@ file MapDrawer.h
@ date 2018-01-29
@ author ukcheol shin
@ brief Frame에 대한 header 파일
**********************************************************************/
#ifndef __MAPDRAWER_H__
#define __MAPDRAWER_H__

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>

#include <mutex>

namespace F_SLAM{
class Frame;
class Tracking;
class FrameDrawer;
class MapDrawer{
public:

    // Create a viz window
    cv::viz::Viz3d *mp_visualizer;

    std::vector<cv::viz::WCameraPosition> cam;
    std::vector<cv::viz::WCameraPosition> cam_cor;
    int numFrame;

    MapDrawer() : numFrame(0)
    {
      mp_visualizer = new cv::viz::Viz3d ("Viz window");
      //  m0_visualizer->setBackgroundColor(cv::viz::Color::white());
    
      cam.push_back(cv::viz::WCameraPosition(cv::Vec2f(0.889484, 0.523599),10.0,cv::viz::Color::black()));
      cam_cor.push_back(cv::viz::WCameraPosition(10));

      mp_visualizer->showWidget("Camera", cam[numFrame]);
      mp_visualizer->showWidget("Camera_cor", cam_cor[numFrame]);
      numFrame++;

      mp_visualizer->spinOnce(1, true);
    }

#if 1 
    ///@brief 현재 frame의 포즈를 받아서 drawing
    ///@todo input parameter를 프레임의 참조자 or 포인터를 받아서 접근하도록.
    void Draw_Fr(cv::Mat &R, cv::Mat &t)
    {
        cv::Affine3d pose(R,t);
        char camname[30], camname2[30]; 
      cam.push_back(cv::viz::WCameraPosition(cv::Vec2f(0.889484, 0.523599),10.0,cv::viz::Color::black()));
      cam_cor.push_back(cv::viz::WCameraPosition(10));

        sprintf(camname, "Camera%d", numFrame);
        sprintf(camname2, "Camera_cor%d", numFrame);
       
        mp_visualizer->showWidget(camname, cam[numFrame]);
        mp_visualizer->showWidget(camname2, cam_cor[numFrame]);
        mp_visualizer->setWidgetPose(camname, pose);
        mp_visualizer->setWidgetPose(camname2, pose);
        numFrame++;
    }

    ///@brief 구해진 점들을 3D 맵에 표현.
    void Draw_Points(std::vector<cv::Vec3d> &points3D)
    {
        #if 0
        // choose one point for visualization
        cv::Vec3d testPoint = triangulate(projection1, projection2, inlierPts1[124], inlierPts2[124]);
        cv::viz::WSphere point3D(testPoint, 0.05, 10, cv::viz::Color::red());
        // its associated line of projection
        double lenght(6.);
        cv::viz::WLine line1(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*inlierPts1[124](0), lenght*inlierPts1[124](1), lenght), cv::viz::Color::green());
        cv::viz::WLine line2(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*inlierPts2[124](0), lenght*inlierPts2[124](1), lenght), cv::viz::Color::green());
        //mp_visualizer->showWidget("Line1", line1);
        //mp_visualizer->showWidget("Triangulated", point3D);
        #endif
        for(int i= 0; i< points3D.size(); i++)
            std::cout << points3D[i] << std::endl;
        // the reconstructed cloud of 3D points
        cv::viz::WCloud cloud(points3D, cv::viz::Color::blue());
        cloud.setRenderingProperty(cv::viz::POINT_SIZE, 3.);

        // Add the virtual objects to the environment
        mp_visualizer->showWidget("Cloud", cloud);
    }

    void Run()
    {
        mp_visualizer->spinOnce(1,true); // redraw
       // std::cout << "running" << std::endl;
    }

    #endif
    }; // end MapDrawer class
} // end namespace

#endif