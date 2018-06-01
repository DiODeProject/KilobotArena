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
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QDebug>


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

KilobotOverheadController::KilobotOverheadController(QObject *parent) : QObject(parent), device(2), sending(false), connected(false)
{
    lastMsgTime.start();
    // OHC link setup
    serial_conn = new SerialConnection();

    connect(serial_conn, SIGNAL(error(QString)), this, SLOT(showError(QString)));
    connect(serial_conn, SIGNAL(status(QString)), this, SLOT(serialUpdateStatus(QString)));

    // Create thread
    QThread *thread = new QThread();
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    // Move connection to thread
    serial_conn->moveToThread(thread);

    // Start thread and open connections
    thread->start();
    serial_conn->open();

    timer.setInterval(20);
    connect(&timer,SIGNAL(timeout()), this, SLOT(sendBatch()));
    timer.start();

    qDebug() << serial_conn->enumerate();
}

KilobotOverheadController::~KilobotOverheadController()
{
    // nothing doing here...
}

void KilobotOverheadController::identifyKilobot(uint8_t id)
{
    assert(id <= pow(2, uint8_t_LENGTH) - 1);

    // TEMPORARY SIGNALLING:
    uint8_t type = (uint8_t) 4;
    uint8_t data_ohc[9] = {(uint8_t) id,0,0,0,0,0,0,0,0};
    this->sendDataMessage(data_ohc, type);
}

void KilobotOverheadController::signalKilobot(kilobot_message message)
{

    // NOTES: -type goes from 0-127 for user defined types and is 8bit unsigned int (128+ reserved for system types)
    //        -data must be an array of 9 unsigned 8bit ints

    // push message onto queue for sending (QTimer will send)
    this->message_q.push_back(message);
    // qDebug() << "Queued message id" << message.id << "type" << message.type << "data" << message.data;
}

void KilobotOverheadController::sendBatch()
{
    //qDebug() << "Running sendBatch()" << this->lastMsgTime.currentTime();
    if (message_q.empty() && !sending && this->lastMsgTime.elapsed() > 50) {
        stopSending();
    }
    while (message_q.size() > 0) {
        this->lastMsgTime.restart();
        if (message_q.size() > 2) {
            uint8_t type = 0; // reserved for three-in-one messages
            uint8_t data[9] = {0,0,0,0,0,0,0,0,0}; // intialise to zero

            // pack data into buffer for each of the three messages (3 bytes for each message, 9 bytes total)
            for (int i = 0; i < 3; ++i) {
                data[i*3] = data[i*3] | (this->message_q.front().id >> 2);
                data[1+i*3] = data[1+i*3] | (this->message_q.front().id << 6);
                data[1+i*3] = data[1+i*3] | (this->message_q.front().type << 2);
                data[1+i*3] = data[1+i*3] | (this->message_q.front().data >> 8);
                data[2+i*3] = data[2+i*3] | this->message_q.front().data;
                // remove message from queue
                this->message_q.pop_front();
            }

            // send message
            this->sendDataMessage(data, type);


        } else if (message_q.size() > 0) {
            uint8_t type = 0; // reserved for three-in-one messages
            uint8_t data[9] = {0,0,0,0,0,0,0,0,0}; // intialise to zero

            for (int i = 0; i < 3; ++i) {
                if (message_q.size() > 0) {
                    data[i*3] = data[i*3] | (this->message_q.front().id >> 2);
                    data[1+i*3] = data[1+i*3] | (this->message_q.front().id << 6);
                    data[1+i*3] = data[1+i*3] | (this->message_q.front().type << 2);
                    data[1+i*3] = data[1+i*3] | (this->message_q.front().data >> 8);
                    data[2+i*3] = data[2+i*3] | this->message_q.front().data;
                    // remove message from queue
                    this->message_q.pop_front();
                } else {
                    kilobot_message msg;
                    msg.id = 1023;
                    msg.type = 0;
                    msg.data = 0;
                    data[i*3] = data[i*3] | (msg.id >> 2);
                    data[1+i*3] = data[1+i*3] | (msg.id << 6);
                    data[1+i*3] = data[1+i*3] | (msg.type << 2);
                    data[1+i*3] = data[1+i*3] | (msg.data >> 8);
                    data[2+i*3] = data[2+i*3] | msg.data;
                }
            }

            // send message
            this->sendDataMessage(data, type);
        }
    }
}

void KilobotOverheadController::broadcastMessage(kilobot_broadcast message)
{
    // NOTES: -type goes from 0-127 for user defined types and is 8bit unsigned int (128+ reserved for system types)
    //        -data must be an array of 9 unsigned 8bit ints

    if (message.type == 0) {
        qDebug() << "Warning - tried to send a user broadcast message with type 0 reserved for composite messages";
        return;
    }

    if (message.type > 127 && message.type != COMMAND_STOP) {
        qDebug() << "Warning - tried to send a user broadcast message with type in the system message range > 127";
        return;
    }

    if (message.type == COMMAND_STOP) {
        this->stopSending();
        //qDebug() << "STOP!";
        return;
    }


    if (message.data.isEmpty()) {
        uint8_t data[9] = {0,0,0,0,0,0,0,0,0};
        this->sendDataMessage(data, message.type);
        sending=true;
    } else {
        this->sendDataMessage(&message.data[0], message.type);
        sending=true;
    }
    //qDebug() << "Broadcasting" << message.type << " content" << message.data;
}


void KilobotOverheadController::serialUpdateStatus(QString str)
{
    serial_status = str;
    updateStatus();
}

void KilobotOverheadController::updateStatus()
{
    QString str = serial_status;
     if (str.startsWith("connect")) {
         connected = true;
         emit errorMessage("OHC connected");
         // enable stuff for when connected
     } else {
         connected = false;
         // disable stuff for when not connected
         emit errorMessage("OHC disconnected");
     }
}

void KilobotOverheadController::toggleConnection() {

    if (serial_status.startsWith("connect")) {
        serial_conn->close();
        emit setStopButton(true);
    }
    else
        serial_conn->open();

}

void KilobotOverheadController::stopSending() {
   sendMessage(COMMAND_STOP);
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
            //return;
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

    serial_conn->resetDelay();
    serial_conn->sendCommand(packet);

}

void KilobotOverheadController::sendDataMessage(uint8_t *payload, uint8_t type) {
    //if (sending) {
    emit setStopButton(true);
        //stopSending();
    //}

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
    //sending = true; //added recently

    serial_conn->queueCommand(packet);
}

void KilobotOverheadController::chooseProgramFile() {
    QSettings settings;
    QString lastDir = settings.value("progLastDir", QDir::homePath()).toString();
    QString filename = QFileDialog::getOpenFileName((QWidget *) sender(), "Open Program File", lastDir, "Program Hex File (*.hex)"); //launches File Selector
    program_file = filename;
    //upload_button->setEnabled(false);
    if (filename.isEmpty()) {
        ((QPushButton *)sender())->setText("[select file]");
    } else {
        QFileInfo info(filename);
        if (info.isReadable()) {
            ((QPushButton *)sender())->setText(info.fileName());
            //if (connected)
            //    upload_button->setEnabled(true);
            QDir dirName (filename);
            dirName.cdUp();
            settings.setValue ("progLastDir", dirName.absolutePath());
        }
        else {
            QMessageBox::critical((QWidget *) sender(), "Kilobots Toolkit", "Unable to open program file for reading.");
            ((QPushButton *)sender())->setText("[select file]");
            program_file = "";
        }
    }
}

void KilobotOverheadController::uploadProgram() {
    if (sending) {
        stopSending();
        emit setStopButton(true);
    }
    if (program_file.isEmpty()) {
         QMessageBox::critical((QWidget *) sender(), "Kilobots Toolkit", "You must select a program file to upload.");
    }
    else {
        // set to boot
        this->sendMessage(BOOT);
        sending = true;
        serial_conn->sendProgram(program_file);
    }
}


void KilobotOverheadController::showError(QString str)
{
    emit errorMessage(str);
}

void KilobotOverheadController::resetKilobots()
{
    sendMessage(RESET);
}

void KilobotOverheadController::sleepKilobots()
{
    sendMessage(SLEEP);
}

void KilobotOverheadController::runKilobots()
{
    sendMessage(RUN);
}

void KilobotOverheadController::checkVoltage()
{
    sendMessage(VOLTAGE);
}

//void KilobotOverheadController::setSerial()
//{
    //this->device = 2;
    /*QVector<QString> ports = SerialConnection::enumerate();
    if (ports.size()>0) {

    }*/
//}
