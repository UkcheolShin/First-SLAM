#ifndef KEYFRAME_H
#define KEYFRAME_H

#endif // KEYFRAME_H

# include <mutex>


namespace kbSLAM {

class frame;
class map;
class keyFrame{
public:
    keyFrame(frame &f,map* pMap);

};

}
