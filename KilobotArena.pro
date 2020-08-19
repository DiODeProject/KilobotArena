#-------------------------------------------------
#
# Project created by QtCreator 2016-10-04T15:07:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = KilobotArena
TEMPLATE = app

# switch between OpenCV2 and 3
CONFIG += use_opencv3
QMAKE_CXXFLAGS += -std=c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    clicksignalqlabel.cpp \
    dragzoomqlabel.cpp \
    kilobottracker.cpp \
    kilobot.cpp \
    kilobotoverheadcontroller.cpp \
    ohc/calibrate.cpp \
    ohc/intelhex.cpp \
    ohc/serialconn.cpp \
    ohc/serialwin.cpp \
    kilobotidassignment.cpp \
    kilobotcalibrate.cpp \
    kilobotcalibrateenv.cpp

HEADERS  += mainwindow.h \
    clicksignalqlabel.h \
    dragzoomqlabel.h \
    kilobottracker.h \
    kilobot.h \
    kilobotoverheadcontroller.h \
    ohc/calibrate.h \
    ohc/intelhex.h \
    ohc/packet.h \
    ohc/serialconn.h \
    ohc/serialwin.h \
    kilobotenvironment.h \
    kilobotexperiment.h \
    userthread.h \
    kilobotidassignment.h \
    kilobotcalibrate.h \
    kilobotcalibrateenv.h

FORMS    += mainwindow.ui

linux {



!use_opencv3 {

# use pkgconfig instead
CONFIG += link_pkgconfig
PKGCONFIG += opencv

}KilobotArena.pro

use_opencv3 {

DEFINES += USE_OPENCV3

# OpenCV 3
#INCLUDEPATH += /opt/local/include/
#LIBS += -L/opt/local/lib \
INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib/ \
        -lopencv_ocl \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_features2d\
        -lopencv_highgui\
#        -lopencv_contrib\
        -lopencv_calib3d\
        -lopencv_objdetect\
        -lopencv_photo\
        -lopencv_stitching\
        -lopencv_flann\
 #       -lopencv_gpu \
 #       -lopencv_legacy \
        -lopencv_ml \
        -lopencv_objdetect  \
 #       -lopencv_ocl \
        -lopencv_photo \
        -lopencv_stitching \
        -lopencv_superres \
  #      -lopencv_ts \
        -lopencv_video \
        -lopencv_videostab \
        -lopencv_videoio \
        -lopencv_imgcodecs \
        -lz
# CUDA
LIBS += -lopencv_cudawarping \
        -lopencv_cudafeatures2d \
        -lopencv_cudaimgproc \
        -lopencv_cudafilters \
        -lopencv_cudaarithm
}


     #-lusb

}

macx {

# Change libstdc++ to C++2011 for OpenCV
CONFIG += c++11

# OpenCV library setup for OSX
INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib \
     -lopencv_core \
     -lopencv_imgproc \
     -lopencv_features2d\
     -lopencv_highgui\
     -lopencv_contrib\
     -lopencv_calib3d\
     -lopencv_objdetect\
     -lopencv_photo\
     -lopencv_stitching\
     -lopencv_flann\
     -lopencv_nonfree\
     -lz

# OpenCV 3rd party libraries
LIBS += /usr/local/share/OpenCV/3rdparty/lib/liblibjpeg.a
LIBS += /usr/local/share/OpenCV/3rdparty/lib/liblibpng.a
LIBS += /usr/local/share/OpenCV/3rdparty/lib/liblibtiff.a
LIBS += /usr/local/share/OpenCV/3rdparty/lib/liblibjasper.a
LIBS += /usr/local/share/OpenCV/3rdparty/lib/libIlmImf.a

# Required for OpenCV
LIBS += -framework AppKit
LIBS += -framework QtKit
LIBS += -framework CoreMedia
LIBS += -framework CoreVideo
LIBS += -framework AVFoundation



}

RESOURCES += \
    ressources.qrc
