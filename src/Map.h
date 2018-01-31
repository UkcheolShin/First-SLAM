#ifndef __MAP_H__
#define __MAP_H__

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace F_SLAM
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map():mnMaxKFid(0) {}

    //Setter
    ///@brief keyframe set에 keyframe pointer 추가
    void AddKeyFrame(KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF); 
        if(pKF->mnId>mnMaxKFid) // 무슨 용도지 이거?
            mnMaxKFid=pKF->mnId;
    }

    ///@brief Mappoint set에 map point pointer 추가
    void AddMapPoint(MapPoint* pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    ///@brief Mappoint Set에서 Map point pointer 제거
    void EraseMapPoint(MapPoint* pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
        // free(pMp); or delete pMp
    }

    ///@brief Keyframe Set에서 Keyframe pointer 제거
    void EraseKeyFrame(KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);
    }

    ///@brief Reference Mappoint를 설정
    // 어디다 쓰는거지 이거;
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    //getter
    ///@brief keyframe Set에서 반복자 구간(시작~ 끝)까지 초기화된 벡터를 반환.
    std::vector<KeyFrame*> GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    }

    ///@brief MapPoint Set에서 반복자 구간(시작~끝)까지 초기화된 벡터를 반환
    std::vector<MapPoint*> GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }

    ///@brief Reference Map point를 반환
    std::vector<MapPoint*> GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    ///@brief Mappoint의 Size를 반환
    long unsigned int MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    ///@brief KeyFrame의 Size를 반환 
    long unsigned  KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    ///@brief Max KeyFrmae id를 반환 (?)
    long unsigned int GetMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void clear()
    {
        for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
            delete *sit;

        for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap; // Map class 접근 mutex
};

} //namespace ORB_SLAM

#endif // MAP_H
