


#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
//#include "LocalMapping.h"
//#include "LoopClosing.h"
//#include "KeyFrameDatabase.h"
//#include "ORBVocabulary.h"
//#include "Viewer.h"

namespace F_SLAM
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const eSensor sensor, const bool bUseViewer = true) :mSensor(sensor)
    {
        // Output welcome message
        cout << endl <<
        "F_SLAM Copyright (C) 2018 UkCheolShin, KAIST." << endl <<
        "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
        "This is free software, and you are welcome to redistribute it" << endl <<
        "under certain conditions. See LICENSE.txt." << endl << endl;

        cout << "Input sensor was set to: ";

        if(mSensor==MONOCULAR)
            cout << "Monocular" << endl;
        else if(mSensor==STEREO)
            cout << "Stereo" << endl;
        else if(mSensor==RGBD)
            cout << "RGB-D" << endl;

        //Create the Map
        mpMap = new Map();

        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
/*
        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
*/
        //Initialize the Viewer thread and launch
        if(bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

/*
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
*/
    }

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp)
    {

        if(mSensor!=MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }

        cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    // Input sensor
    eSensor mSensor;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace F_SLAM

#endif // SYSTEM_H
