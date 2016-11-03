#ifndef KILOBOTOVERHEADCONTROLLER_H
#define KILOBOTOVERHEADCONTROLLER_H

#include <QObject>
#include <assert.h>
#include <math.h>
#include "kilobot.h"

class KilobotOverheadController : public QObject
{
    Q_OBJECT
public:
    explicit KilobotOverheadController(QObject *parent = 0);
    ~KilobotOverheadController();
    void assignIDs(); // assign IDs
    //getIDs(); // return list of ids
    void identifyKilobot(kilobot_id id);
    void signalKilobot(kilobot_id id, kilobot_message_type message, kilobot_message_data data);

signals:

public slots:
};


#endif // KILOBOTOVERHEADCONTROLLER_H
