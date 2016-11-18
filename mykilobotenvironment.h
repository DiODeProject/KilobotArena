#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
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
    void generateEnviron(Kilobot*);

};
#endif // MYKILOBOTENVIRONMENT_H
