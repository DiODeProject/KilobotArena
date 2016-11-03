#-------------------------------------------------
#
# Project created by QtCreator 2016-10-04T15:07:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = KilobotArena
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    clicksignalqlabel.cpp \
    dragzoomqlabel.cpp \
    kilobottracker.cpp \
    kilobot.cpp \
    kilobotoverheadcontroller.cpp \
    ohc/calibrate.cpp \
    ohc/ftdiconn.cpp \
    ohc/intelhex.cpp \
    ohc/serialconn.cpp \
    ohc/serialwin.cpp \
    ohc/vusbconn.cpp

HEADERS  += mainwindow.h \
    clicksignalqlabel.h \
    dragzoomqlabel.h \
    kilobottracker.h \
    kilobot.h \
    kilobotoverheadcontroller.h \
    ohc/calibrate.h \
    ohc/ftdiconn.h \
    ohc/intelhex.h \
    ohc/packet.h \
    ohc/serialconn.h \
    ohc/serialwin.h \
    ohc/vusbconn.h

FORMS    += mainwindow.ui

linux {

CONFIG += link_pkgconfig
PKGCONFIG += libftdi1
PKGCONFIG += libusb
PKGCONFIG += opencv

# OpenCV 2
#INCLUDEPATH += /usr/local/include
#LIBS += -L/usr/local/lib \
#     -lopencv_core \
#     -lopencv_imgproc \
#     -lopencv_features2d\
#     -lopencv_highgui\
#     -lopencv_contrib\
#     -lopencv_calib3d\
#     -lopencv_objdetect\
#     -lopencv_photo\
#     -lopencv_stitching\
#     -lopencv_flann\
#     -lz
#     -lusb

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

}
