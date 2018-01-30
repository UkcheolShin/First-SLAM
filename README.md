## First-SLAM
This is an OpenCV 3.0 based implementation of a monocular visual odometry algorithm.

## Algorithm
Uses Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker.

reference
1.[here as a report](http://avisingh599.github.io/assets/ugp2-report.pdf)
2.[here as a blog post](http://avisingh599.github.io/vision/monocular-vo/). 

Note that this project is not yet capable of doing reliable relative scale estimation, 
so the scale informaion is extracted from the KITTI dataset ground truth files.

## To do
1. Two view mapping in single task ( due data : 2018.01.23)
> 1. Make up Map point class
> 2. Make up Map class
> 3. triangulation (done)
> 4. visualize 3D map (done)
> 5. Make up frame class 
> 6. complement 3 task (2d drawer, 3d drawer, tracking)
> 7. make system.cpp 
> 8. key-frame selection
> 9. key-frame class
> 10. BA class
> 11. Local Mapping Class 


## Demo Video
[![Demo video](http://share.gifyoutube.com/Ke1ope.gif)](http://www.youtube.com/watch?v=homos4vd_Zs)

## Requirements
OpenCV 3.0

## How to compile?
Provided with this repo is a CMakeLists.txt file, which you can use to directly compile the code as follows:
```bash
mkdir build
cd build
cmake ..
make
```

## How to run? 
After compilation, in the build directly, type the following:
```bash
./vo
```

## Before you run
In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.

## Performance
![Results on the KITTI VO Benchmark](http://avisingh599.github.io/images/visodo/2K.png)

## Contact
For any queries, contact: shinwc159@gmail.com or parkkibaek@kaist.ac.kr

## License
MIT
