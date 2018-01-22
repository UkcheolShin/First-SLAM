#ifndef KEYFRAME_H
#define KEYFRAME_H


# include <mutex>


namespace F_SLAM {

class Frame;
class Map;
class KeyFrame{
public:
    KeyFrame(frame &f,map* pMap);

};

}


#endif // KEYFRAME_Hs