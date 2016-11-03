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
    void errorMessage(QString);

public slots:
    void serialUpdateStatus(QString);
    void ftdiUpdateStatus(QString);
    void vusbUpdateStatus(QString);
    void showError(QString);
    void toggleConnection();

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

    // PRIVATE METHODS

    void updateStatus();

    void stopSending();
    void sendMessage(int);
    void sendDataMessage(uint8_t *, uint8_t);

};


#endif // KILOBOTOVERHEADCONTROLLER_H
