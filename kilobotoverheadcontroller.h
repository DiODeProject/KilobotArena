#ifndef KILOBOTOVERHEADCONTROLLER_H
#define KILOBOTOVERHEADCONTROLLER_H

#include <QObject>
#include <assert.h>
#include <math.h>
#include "kilobot.h"
#include <QList>
#include <QPushButton>

// Include for communication with the OHC
#include <stdint.h>
#include "ohc/serialwin.h"
#include "ohc/vusbconn.h"
#include "ohc/ftdiconn.h"
#include "ohc/serialconn.h"
#include "ohc/calibrate.h"

typedef unsigned int goal_pos;

class KilobotOverheadController : public QObject
{
    Q_OBJECT
public:
    explicit KilobotOverheadController(QObject *parent = 0);
    ~KilobotOverheadController();
    void assignIDs(); // assign IDs
    //getIDs(); // return list of ids

signals:
    void errorMessage(QString);
    void setStopButton(bool);

public slots:
    void identifyKilobot(kilobot_id id);
    void broadcastMessage(kilobot_message_type message, kilobot_message_data data);
    void signalKilobot(kilobot_id id, kilobot_message_type message, QVector <uint8_t> data);
    void serialUpdateStatus(QString);
    void ftdiUpdateStatus(QString);
    void vusbUpdateStatus(QString);
    void showError(QString);

    // CONTROL
    void toggleConnection(); // connect / disconnect

    // access the build in kilobot commands for all kilobots
    void resetKilobots();
    void sleepKilobots();
    void runKilobots();
    void stopSending();

    void chooseProgramFile(); // find program file
    void uploadProgram(); // upload program file

    void addGoal(); // add goal for kilobots

private:

    // PRIVATE MEMBERS

    int device;
    bool sending;
    bool connected;

    VUSBConnection *vusb_conn;
    FTDIConnection *ftdi_conn;
    SerialConnection *serial_conn;

    QString vusb_status;
    QString ftdi_status;
    QString serial_status;

    QString program_file;

    goal_pos x_g = 0;
    goal_pos y_g = 0;

    // PRIVATE METHODS

    void updateStatus();

    void sendMessage(int);
    void sendDataMessage(uint8_t *, uint8_t);

};


#endif // KILOBOTOVERHEADCONTROLLER_H
