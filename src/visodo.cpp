#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include <thread>

using namespace cv;
using namespace std;

#define MAX_FRAME 4540

/**
@ brief main함수.
@ todo input img가 3채널, rgb일경우 gray로 바꾸게 하는 코드 추가할 것.
*/
int main( int argc, char** argv )   {

  cv::Mat img_track;        
  cv::Point3d xyz;
  char filename[100];     // image file read 용도
  std::ofstream fd_result;     // 결과를 저장할 file descriptor  
  double scale = 1.00;    // 기본 스케일은 1로 
  cv::Mat traj = Mat::zeros(600, 600, CV_8UC3);
  cv::Mat R,t;

  std::cout <<"0.Operate Main function..."<< std::endl;
  sprintf(filename, "/media/sf_VirtualBox_share0/data_odometry_gray/dataset/sequences/00/image_0/%06d.png", 0);

  // 1.read the first two frames from the dataset
  cv::Mat img = imread(filename);
  F_SLAM::Tracking visu_odo(img);
  visu_odo.Initialize_Tracking();
  F_SLAM::FrameDrawer drawer;
  F_SLAM::MapDrawer m_drawer;

  // 2.undistroted image
  // kitti data set은 이미 적용되어 있어서 여기선 생략됨

  clock_t begin = clock();
  fd_result.open ("results1_1.txt"); // 결과를 쓸 파일 생성.

  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Tracked image", WINDOW_AUTOSIZE );// Create a window for display.

//  int numFrame = 0;
//  while(!visualizer.wasStopped()){
  for(int numFrame=1; numFrame < MAX_FRAME; numFrame++)   {
//   numFrame++;
    // 1. read image
    sprintf(filename, "/media/sf_VirtualBox_share0/data_odometry_gray/dataset/sequences/00/image_0/%06d.png", numFrame);
    img = imread(filename);
    visu_odo.SetCurImage(img);

    // 2.feature tracking
    visu_odo.featureTracking();

    // 3.find Essential & Pose
    cv::Mat inliers;
    visu_odo.calcPose(inliers);
    
    int numberOfPts = cv::sum(inliers)[0];
    std::cout << "Number of inliers: " << numberOfPts << std::endl;  

    // 4. 스케일 가져옴
    scale = visu_odo.getAbsoluteScale(numFrame, xyz);
    std::cout << "Scale is " << scale << std::endl;
 
    if (scale>0.1) {
      visu_odo.w2c_pose(scale,R,t);
    }
    else {
     cout << "scale below 0.1, or incorrect translation" << endl;
    }
    
   // lines for printing results
    fd_result << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2) << endl;
    
    std::vector<cv::Vec3d> points3D ;
    visu_odo.triangulation(R, t, points3D); 
    //m_drawer.Draw_Fr(R,t);
    //m_drawer.Draw_Points(points3D);
    
    //Use a member function in a thread
    std::thread th1(&F_SLAM::MapDrawer::Draw_Fr,&m_drawer,std::ref(R),std::ref(t));
    std::thread th2(&F_SLAM::MapDrawer::Draw_Points,&m_drawer,std::ref(points3D));
//    thread th3(&F_SLAM::MapDrawer::Run,&m_drawer);

    //Join the thread with the main thread
    th1.join();
    th2.join();   
//    th3.join();
    m_drawer.Run();

   // 5. a redetection is triggered in case the number of feautres being trakced go below a particular threshold
    visu_odo.checkKeyPoint();

    visu_odo.handleTrackedPoints(img_track);

    visu_odo.update();

    drawer.odometry_draw2D(t,xyz,traj);

//    imshow("Road facing camera", img_curr );
    imshow("Trajectory", traj );
    imshow("Tracked image", img_track);

//    while(1)
//      m_drawer.Run();
    waitKey(1);
//  }
}
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  cout << R << endl;
  cout << t << endl;

  return 0;
}
