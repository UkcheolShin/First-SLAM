QT += core
QT -= gui

CONFIG += c++11

TARGET = mono_vo_qt
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    src/visodo.cpp

HEADERS += \
    src/vo_features.h \
    src/keyframe.h \
    src/frame.h \
    src/header.h \
    src/tracking.h


QMAKE_CXXFLAGS += -std=c++11


INCLUDEPATH += /usr/include/boost
INCLUDEPATH += /usr/include/opencv/
INCLUDEPATH += /usr/include/eigen3/
INCLUDEPATH += /usr/include/eigen3/Eigen/
INCLUDEPATH += /home/rcv-exp/ceres-solver/include/


LIBS += -L/usr/local/lib
LIBS += -L/usr/lib
LIBS += `pkg-config opencv –cflags –libs`
LIBS += -lopencv_core
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_videoio
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
#LIBS += -lopencv_nonfree
LIBS += -lceres -lcsparse -lcholmod -fopenmp -lcblas -lblas -llapack
LIBS += -lboost_system
