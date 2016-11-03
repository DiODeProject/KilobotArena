/*
 * KilobotOverheadController
 *
 * Created: 3rd Nov 2016
 * Alex Cope
 *
 * Mainly adapted from the KiloGUI example code, but allowing integration with the Smart Arena
 * infrastructure
 *
*/

#include "kilobotoverheadcontroller.h"
#include "ohc/packet.h"

#include <QThread>

// OHC data structures & defs
typedef struct {
    const char *name;
    const unsigned char type;
} kilo_cmd_t;

#define COMMAND_STOP 250
#define COMMAND_LEDTOGGLE 251

static const kilo_cmd_t KILO_COMMANDS[] = {
    {"Reset", RESET},
    {"Run", RUN},
    {"Pause", WAKEUP},
    {"Sleep", SLEEP},
    {"Voltage", VOLTAGE},
    {"LedToggle", COMMAND_LEDTOGGLE},
    {"Charging", CHARGE}
};

static const int NUM_KILO_COMMANDS = sizeof(KILO_COMMANDS)/sizeof(kilo_cmd_t);

KilobotOverheadController::KilobotOverheadController(QObject *parent) : QObject(parent), device(0), sending(false), connected(false)
{

    // OHC link setup
    vusb_conn = new VUSBConnection();
    ftdi_conn = new FTDIConnection();
    serial_conn = new SerialConnection();
    //connect(ftdi_conn, SIGNAL(readText(QString)), serial, SLOT(addText(QString)));
    //connect(serial_conn, SIGNAL(readText(QString)), serial, SLOT(addText(QString)));

    connect(vusb_conn, SIGNAL(error(QString)), this, SLOT(showError(QString)));
    connect(vusb_conn, SIGNAL(status(QString)), this, SLOT(vusbUpdateStatus(QString)));
    connect(ftdi_conn, SIGNAL(error(QString)), this, SLOT(showError(QString)));
    connect(ftdi_conn, SIGNAL(status(QString)), this, SLOT(ftdiUpdateStatus(QString)));
    connect(serial_conn, SIGNAL(error(QString)), this, SLOT(showError(QString)));
    connect(serial_conn, SIGNAL(status(QString)), this, SLOT(serialUpdateStatus(QString)));

    // Create thread
    QThread *thread = new QThread();
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    // Move connections to thread
    vusb_conn->moveToThread(thread);
    ftdi_conn->moveToThread(thread);
    serial_conn->moveToThread(thread);

    // Start thread and open connections
    thread->start();
    vusb_conn->open();
    ftdi_conn->open();
    serial_conn->open();
}

KilobotOverheadController::~KilobotOverheadController()
{
    // nothing doing here...
}

void KilobotOverheadController::identifyKilobot(kilobot_id id)
{
    assert(id <= pow(2, KILOBOT_ID_LENGTH) - 1);
}

void KilobotOverheadController::signalKilobot(kilobot_id id, kilobot_message_type message, kilobot_message_data data)
{
    assert(id <= pow(2, KILOBOT_ID_LENGTH) - 1);
    assert(message <= pow(2, KILOBOT_MESSAGE_TYPE_LENGTH) - 1);
    assert(data <= pow(2, KILOBOT_MESSAGE_DATA_LENGTH) - 1);

    // TODO this method should work on a queue basis - signals should be queued until at least 3 are available, then broadcast in a single message
}

void KilobotOverheadController::ftdiUpdateStatus(QString str)
{
    ftdi_status = str;
    updateStatus();
}

void KilobotOverheadController::vusbUpdateStatus(QString str)
{
    vusb_status = str;
    updateStatus();
}

void KilobotOverheadController::serialUpdateStatus(QString str)
{
    serial_status = str;
    updateStatus();
}

void KilobotOverheadController::updateStatus()
{
    // display some info
}

void KilobotOverheadController::toggleConnection() {
    if (device == 0) {
        if (ftdi_status.startsWith("connect"))
            ftdi_conn->close();
        else
            ftdi_conn->open();
    } else if (device == 1) {
        if (vusb_status.startsWith("connect"))
            vusb_conn->close();
        else
            vusb_conn->open();
    } else {
        if (serial_status.startsWith("connect"))
            serial_conn->close();
        else
            serial_conn->open();
    }
}

void KilobotOverheadController::stopSending() {
    if (sending)
        sendMessage(COMMAND_STOP);
    /*QList<QPushButton*>::iterator i;
    for (i = toggle_buttons.begin(); i != toggle_buttons.end(); i++) {
        if ((*i)->isChecked())
            (*i)->setChecked(false);
    }*/
}

void KilobotOverheadController::sendMessage(int type_int) {
    unsigned char type = (unsigned char)type_int;
    QByteArray packet(PACKET_SIZE, 0);
    if (type == COMMAND_STOP) {
        sending = false;
        packet[0] = PACKET_HEADER;
        packet[1] = PACKET_STOP;
        packet[PACKET_SIZE-1]=PACKET_HEADER^PACKET_STOP;
    } else  {
        if (sending) {
            stopSending();
            return;
        }

        if (type == COMMAND_LEDTOGGLE) {
            sending = false;
            packet[0] = PACKET_HEADER;
            packet[1] = PACKET_LEDTOGGLE;
            packet[PACKET_SIZE-1]=PACKET_HEADER^PACKET_LEDTOGGLE;
        } else {
            sending = true;
            packet[0] = PACKET_HEADER;
            packet[1] = PACKET_FORWARDMSG;
            packet[11] = type;
            packet[PACKET_SIZE-1]=PACKET_HEADER^PACKET_FORWARDMSG^type;
        }
    }

    if (device == 0)
        ftdi_conn->sendCommand(packet);
    else if (device == 1)
        vusb_conn->sendCommand(packet);
    else
        serial_conn->sendCommand(packet);
}

void KilobotOverheadController::sendDataMessage(uint8_t *payload, uint8_t type) {
    if (sending)
        stopSending();

    QByteArray packet(PACKET_SIZE, 0);
    uint8_t checksum = PACKET_HEADER^PACKET_FORWARDMSG^type;
    packet[0] = PACKET_HEADER;
    packet[1] = PACKET_FORWARDMSG;
    for (int i = 0; i < 9; i++) {
        packet[2+i] = payload[i];
        checksum ^= payload[i];
    }
    packet[11] = type;
    packet[PACKET_SIZE-1] = checksum;
    sending = true;


    if (device == 0)
        ftdi_conn->sendCommand(packet);
    else if (device == 1)
        vusb_conn->sendCommand(packet);
    else
        serial_conn->sendCommand(packet);
}

void KilobotOverheadController::showError(QString str)
{
    emit errorMessage(str);
}
