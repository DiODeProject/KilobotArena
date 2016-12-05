#ifndef KILOBOTENVIRONMENT_H
#define KILOBOTENVIRONMENT_H

#include <QObject>
#include <QVector>
#include "kilobot.h"

class KilobotEnvironment : public QObject
{
    Q_OBJECT
public:
    explicit KilobotEnvironment(QObject *) {}
    KilobotEnvironment() {}
//    virtual QVector <uint8_t> getEnvironmentValue(QPointF) { QVector <uint8_t> temp; return temp;} // Don't need?

signals:
    void transmitKiloState(kilobot_message);
    //void broadcastMessage(kilobot_broadcast);

public slots:
    virtual void update() {}
    virtual void updateVirtualSensor(Kilobot) {} // Call this updateVirtualSensor(...)


};

#endif // KILOBOTENVIRONMENT_H
