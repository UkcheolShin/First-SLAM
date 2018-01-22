#include "vo_features.h"
#include "header.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!
// real scale from GT
double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)
{
    string line;
    int i = 0;
    char filename1[200];
    sprintf(filename1, "/home/rcv-exp/workspace/SLAM/kitti_data/dataset/poses/%02d.txt",sequence_id);
    ifstream myfile (filename1);
    double tx =0, ty=0, tz = 0;
    double tx_prev, ty_prev, tz_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id)){
            tz_prev = tz;
            tx_prev = tx;
            ty_prev = ty;
            std::istringstream in(line);
            for (int j=0; j<12; j++)
            {
                in >> tz ;
                if (j==7) ty=tz;
                if (j==3)  tx=tz;
            }
            i++;
        }
        myfile.close();
    }
    else{
        cout << "Unable to open file";
        return 0;
    }

    // real scale about relative pose
    return sqrt((tx-tx_prev)*(tx-tx_prev) + (ty-ty_prev)*(ty-ty_prev) + (tz-tz_prev)*(tz-tz_prev)) ;
}
void getCalib(int sequence_id, double* calib)
{
    string line,line2;
    int i = 0;
    char filename1[200];
    sprintf(filename1, "/home/rcv-exp/workspace/SLAM/kitti_data/dataset/sequences_calib/%02d/calib.txt",sequence_id);
    ifstream myfile (filename1);

    // left color camera : 2
    int camera_num = 2;
    double _calib[3];
    cv::Point3d pp(0.0,0.0, 0.0);
    double tmp;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ))
        {
            if (i==camera_num){
                std::istringstream in(line);
                for (int j=0; j<12; j++){
                    getline(in, line2, ' ');
                    if (j==1) calib[0]= atof(line2.c_str()); // focal
                    if (j==3) calib[1]= atof(line2.c_str()); // cx
                    if (j==7) calib[2] =atof(line2.c_str()); // cy
                }
            }
            i++;
        }
        myfile.close();
    }
    else
        cout << "Unable to open file";
}


int main( int argc, char** argv )
{
    int seqNum = 0;
    // save result
    ofstream myfile;
    myfile.open ("/home/rcv-exp/workspace/SLAM/mono_vo/mono_vo_qt/result/results1_1.txt");

    // Declare
    double scale = 1.00;
    char filename1[200],filename2[200],text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);
    clock_t begin,end;
    double elapsed_secs;
    namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
    namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);

    cv::Mat img_2_c;
    cv::Mat R_f, t_f;
    cv::Mat prevImage,currImage,currImage_c;
    vector<Point2f> prevFeatures,currFeatures;
    cv::Mat E, R, t, mask;
    //TODO: add a fucntion to load these values directly from KITTI's calib files
    // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
    double calib[3];
    getCalib(seqNum,calib);
    double focal = calib[0];
    cv::Point2d pp(calib[1],calib[2]);


  for(int numFrame=0; numFrame < MAX_FRAME; numFrame++)
  {
      begin = clock();
      if(numFrame==0) // The first sequence
      {
          sprintf(filename1, "/home/rcv-exp/workspace/SLAM/kitti_data/dataset/sequences/%02d/image_2/%06d.png",seqNum, 0);
          sprintf(filename2, "/home/rcv-exp/workspace/SLAM/kitti_data/dataset/sequences/%02d/image_2/%06d.png",seqNum, 1);
          currImage_c = imread(filename1); cvtColor(currImage_c, prevImage, COLOR_BGR2GRAY);
          img_2_c = imread(filename2); cvtColor(img_2_c, currImage, COLOR_BGR2GRAY);
          vector<uchar> status;
          featureDetection(prevImage, prevFeatures);
          featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
          E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
          recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
          R_f = R.clone();
          t_f = t.clone();
      }
      else // The other sequence
      {
          sprintf(filename1, "/home/rcv-exp/workspace/SLAM/kitti_data/dataset/sequences/%02d/image_2/%06d.png", seqNum,numFrame);
          currImage_c = imread(filename1); cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
          vector<uchar> status;
          featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
          E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
          recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
          // multiply relative pose
          scale = getAbsoluteScale(numFrame, seqNum, t.at<double>(2));

          // TODO : need to improve threshold
          if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))){
              t_f = t_f + scale*(R_f*t);
              R_f = R*R_f;
          }
          else{
              cout << "scale : " << scale << endl;
              cout << "scale below 0.1, or incorrect translation" << endl;
          }

      }
      if (prevFeatures.size() < MIN_NUM_FEAT) // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
      {
          vector<uchar> status;
          featureDetection(prevImage, prevFeatures);
          featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
      }

#ifdef TRIANGULATION
      // R,t로부터 projection matrix 구성.
      // 한쪽을 [I|0]이라 생각, 다른 한쪽을 [R|t]라 생각.
      cv::Mat projection2(3, 4, CV_64F);        // the 3x4 projection matrix
      R_f.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
      t_f.copyTo(projection2.colRange(3, 4));

      // compose generic projection matrix 
      cv::Mat projection1(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
      cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
      diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));

      std::cout << "First Projection matrix=" << projection1 << std::endl;
      std::cout << "Second Projection matrix=" << projection2 << std::endl;

      // inlier point 들을 구성하여 undistort하고, triangulate함.
      // to contain the inliers
      std::vector<cv::Vec2d> inlierPts1;
      std::vector<cv::Vec2d> inlierPts2;

      // create inliers input point vector for triangulation
      int j(0); 
      for (int i = 0; i < inliers.rows; i++) {

        if (inliers.at<uchar>(i)) {
          inlierPts1.push_back(cv::Vec2d(points_prev[i].x, points_prev[i].y));
          inlierPts2.push_back(cv::Vec2d(points_curr[i].x, points_curr[i].y));
        }
      }
      cv::Mat cameraMatrix; /// Camera Matrix
      cv::Mat distCoeffs; /// Camera Distortion Matrix

      // undistort and normalize the image points
      std::vector<cv::Vec2d> points1u;
      cv::undistortPoints(inlierPts1, points1u, cameraMatrix, cameraDistCoeffs);
      std::vector<cv::Vec2d> points2u;
      cv::undistortPoints(inlierPts2, points2u, cameraMatrix, cameraDistCoeffs);

     // triangulation
      std::vector<cv::Vec3d> points3D;
      triangulate(projection1, projection2, inlierPts1, inlierPts2, points3D);
#endif


      // Update variable
      prevImage = currImage.clone();
      prevFeatures = currFeatures;


      // Save
      myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;
      // Display
      end = clock();
      elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

      int x = int(t_f.at<double>(0)) + 300;
      int y = int(t_f.at<double>(2)) + 100;
      circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);
      rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
      sprintf(text, "x = %02fm y = %02fm z = %02fm FPS = %.02f", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2), 1.0/elapsed_secs);
      putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
      imshow( "Road facing camera", currImage_c );
      imshow( "Trajectory", traj );
//      system("clear");
      waitKey(1);

  }
  return 0;

}
