/**------------------------------------------------------------------------------------------*\
@ file tracking.h
@ date 2018-1-22
@ brief triangulation 함수 추가 
\*------------------------------------------------------------------------------------------*/

#ifndef TRACKING_H
#define TRACKING_H


namespace F_SLAM{
class Tracking{
public :
	// triangulate using Linear LS-Method
	cv::Vec3d triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2);
	void triangulate(const cv::Mat &p1, const cv::Mat &p2, const std::vector<cv::Vec2d> &pts1, const std::vector<cv::Vec2d> &pts2, std::vector<cv::Vec3d> &pts3D);

protected:
    void MonocularInitialization(); // first two view motion
    void CreateInitialMapMonocular();// first two view 3D

};

}


#endif // TRACKING_H
