#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPointF>
#include <QVector>
#include <kilobotenvironment.h>
#include <QTime>
#include <QColor>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//enum command {
//    STRAIGHT_L,
//    LEFT,
//    RIGHT,
//    STOP,
//    LIGHT,
//    STRAIGHT_R,
//    DONE_MOTION
//};
enum command {
    LEFT=1,
    RIGHT,
    STOP,
    STRAIGHT_L,
    STRAIGHT_R,
    DONE_MOTION
};

enum CalibrationStage{
    DETECTING_MOVE,
    EVALUATING_REV_SPEED,
    EVALUATING_STRAIGHT_MOTION,
    DONE
};


class KilobotCalibrateEnv : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit KilobotCalibrateEnv(QObject *parent = 0);
    bool rotDone = false;
    bool strDone = false;
    void setKilobotRadius(double kilobot_radius){ this->kilobotRadius = kilobot_radius; }
    double kilobotRadius;

    QVector < command >  commandLog;
    QVector < QPointF >  velocityLog;
    QVector < QVector < QPointF > >  posLog;
    QVector < QVector < int > > posLogTimes;

signals:
    void drawLine(std::vector<cv::Point> pos, QColor col, int thickness, std::string text, bool transparent);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);

private:

    bool evaluateSpeed(int step, int timeInterval, double robotRadius, int &speed, int index, command motionType);
    double euclideanDist(const cv::Point& a, const cv::Point& b);
//    void rotateVect(QPointF& vector2d, double angleInDeg);
    void sendCalibMessage(kilobot_id kID);

    QVector < bool > isInit;
    QVector < QTime > times;
    QVector < int > noGoodSpeedCounter;
    QVector < QPoint > left_right;
    QVector < CalibrationStage > calibrationStage;

    int stage = 0;
    int num_done = 0;
    int num_kilobots = 0;

};
#endif // MYKILOBOTENVIRONMENT_H
