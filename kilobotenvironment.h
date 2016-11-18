#ifndef KILOBOTENVIRONMENT_H
#define KILOBOTENVIRONMENT_H

#include <QObject>
#include <QVector>
#include "kilobot.h"

class KilobotEnvironment : public QObject
{
    Q_OBJECT
public:
    explicit KilobotEnvironment(QObject *parent = 0);
//    virtual QVector <uint8_t> getEnvironmentValue(kilobot_pos, kilobot_pos) { QVector <uint8_t> temp; return temp;}

signals:
    void transmitKiloState(kilobot_id id, kilobot_message_type message, QVector <uint8_t> data);

public slots:
    virtual void update() {}
    virtual void generateEnviron(Kilobot*) {}


};

#endif // KILOBOTENVIRONMENT_H
