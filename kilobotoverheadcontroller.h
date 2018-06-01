#ifndef KILOBOTOVERHEADCONTROLLER_H
#define KILOBOTOVERHEADCONTROLLER_H

#include <QObject>
#include <assert.h>
#include <math.h>
#include "kilobot.h"
#include <QList>
#include <QPushButton>
#include<QTimer>

// Include for communication with the OHC
#include <stdint.h>
#include "ohc/serialwin.h"
#include "ohc/serialconn.h"
#include "ohc/calibrate.h"



class KilobotOverheadController : public QObject
{
    Q_OBJECT
public:
    explicit KilobotOverheadController(QObject *parent = 0);
    ~KilobotOverheadController();

signals:
    void errorMessage(QString);
    void setStopButton(bool);

public slots:
    void identifyKilobot(uint8_t id);
    void broadcastMessage(kilobot_broadcast);
    void signalKilobot(kilobot_message);
    void serialUpdateStatus(QString);
    void showError(QString);

    // connect / disconnect
    void toggleConnection();

    // access the build in kilobot commands for all kilobots
    void resetKilobots();
    void sleepKilobots();
    void runKilobots();
    void stopSending();
    void checkVoltage();

    // program loading
    void chooseProgramFile();
    void uploadProgram();

    // timed messages
    void sendBatch();

private:

    // PRIVATE MEMBERS

    int device;
    bool sending;
    bool connected;

    SerialConnection *serial_conn;

    QString serial_status;

    QString program_file;

    // PRIVATE METHODS

    void updateStatus();

    void sendMessage(int);
    void sendDataMessage(uint8_t *, uint8_t);

    QVector < kilobot_message > message_q;

    QTimer timer;
    QTime lastMsgTime;

};


#endif // KILOBOTOVERHEADCONTROLLER_H
