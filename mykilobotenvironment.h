#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPoint>
#include "kilobotenvironment.h"

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    mykilobotenvironment(QObject *parent = 0);

//   QVector <uint8_t> getEnvironmentValue(kilobot_pos, kilobot_pos);

signals:

public slots:
    void update();
    void generateEnvironment(Kilobot kilobot);

private:
    QPoint target;

};
#endif // MYKILOBOTENVIRONMENT_H
