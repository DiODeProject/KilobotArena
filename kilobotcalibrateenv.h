#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPointF>
#include <QVector>
#include <kilobotenvironment.h>
#include <QTime>

enum command {
    STRAIGHT_L,
    LEFT,
    RIGHT,
    STOP,
    LIGHT,
    STRAIGHT_R
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

    QVector < QTime > times;
    QVector < int > count;
    QVector < QPoint > left_right;
    QVector < int > dual_offset;
    QVector < command >  commandLog;
    QVector < QPointF >  velocityLog;
    int stage = 0;
    int num_done = 0;

};
#endif // MYKILOBOTENVIRONMENT_H
