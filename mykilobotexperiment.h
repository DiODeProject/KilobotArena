#ifndef MYKILOBOTEXPERIMENT_H
#define MYKILOBOTEXPERIMENT_H

#include <QObject>

//// OpenCV includes
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/video/video.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/videostab/videostab.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/stitching/stitcher.hpp>

//using namespace cv;

// Project includes
#include "kilobot.h"
#include "kilobottracker.h"
#include "kilobotoverheadcontroller.h"
#include "kilobotexperiment.h"
#include "kilobotenvironment.h"

#include "mykilobotenvironment.h"

class mykilobotexperiment : public KilobotExperiment
{
    Q_OBJECT
public:
    explicit mykilobotexperiment(QObject *parent = 0);

    QVector < Kilobot * > mykilobot;

signals:
    void errorMessage(QString);

public slots:
    void initialise(bool);
    void run();
//    void broadcastMessage(kilobot_message);

    void setKBtype1(int);
    void setGoal1(int);
    void setHome1(int);
    void setKBtype2(int);
    void setGoal2(int);
    void setHome2(int);
    void setExperiment1();
    void setExperiment2();
    void setupExperiment();

private:
    void setupInitialKilobotEnvironment(Kilobot kilobotCopy);
    void updateKilobotEnvironment(Kilobot kilobotCopy);
    void updateKilobotVS(Kilobot kilobotCopy);

    //
    void setupEnvironment1();
    void setupEnvironment2();
    void assignKilobotIDs(Kilobot kilobotCopy);

    //
    KilobotTracker mytracker;
    KilobotOverheadController myohc;
    mykilobotenvironment myenviron1;
    mykilobotenvironment myenviron2;

    //
    int exptID = 0;

};

#endif // MYKILOBOTEXPERIMENT_H
