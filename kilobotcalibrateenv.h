#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPointF>
#include <QVector>
#include <kilobotenvironment.h>
#include <QTime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

enum command {
    STRAIGHT_L,
    LEFT,
    RIGHT,
    STOP,
    LIGHT,
    STRAIGHT_R,
    DONE_MOTION
};

enum CalibrationStage{
    DETECTING_MOVE,
    EVALUATING_REV_SPEED,
    DONE
};


class KilobotCalibrateEnv : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit KilobotCalibrateEnv(QObject *parent = 0);
    bool rotDone = false;
    bool strDone = false;

signals:

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);

private:

    bool evaluateSpeed(int step, int timeInterval, double robotRadius, int &speed, int index);
    double euclideanDist(const cv::Point& a, const cv::Point& b);

    QVector < bool > isInit;
    QVector < QTime > times;
    QVector < int > noGoodSpeedCounter;
    QVector < QPoint > left_right;
    QVector < CalibrationStage > calibrationStage;
    QVector < command >  commandLog;
    QVector < QPointF >  velocityLog;
    QVector < QVector < QPointF > >  posLog;
    QVector < QVector < int > > posLogTimes;
    int stage = 0;
    int num_done = 0;
    int num_kilobots = 0;

};
#endif // MYKILOBOTENVIRONMENT_H
