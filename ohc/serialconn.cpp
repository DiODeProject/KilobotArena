#include "serialconn.h"
#include "packet.h"
#include <QDir>
#include <QDebug>
//#include <algorithm>
//#include <unistd.h>

#ifdef _WIN32
// based on http://playground.arduino.cc/Interfacing/CPPWindows
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#else
// based on https://github.com/todbot/arduino-serial
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#endif

static unsigned char buf[4096];
static uint8_t packet[PACKET_SIZE];

enum {
    MODE_NORMAL = 0,
    MODE_UPLOAD = 0x01,
    MODE_DOWNLOAD = 0x02
};

SerialConnection::SerialConnection(QObject *parent, QString _portname): QObject(parent), portname(_portname), context(NULL), mode(MODE_NORMAL)
{ delay.start();

}

QVector<QString> SerialConnection::enumerate() {
    QVector<QString> ports;
#ifdef _WIN32
    COMMCONFIG cc;
    DWORD dwSize = sizeof(COMMCONFIG);
    for (size_t i=1; i<20; i++)
    {
        QString szPort = QString("COM%1").arg(QString::number(i));
        if (GetDefaultCommConfig(szPort.toStdWString().c_str(), &cc, &dwSize))
            ports.push_back(szPort);
    }
#else
    QDir dir("/dev");
    QStringList filters;
    filters << "ttyUSB*" << "ttyACM*" << "tty.usb*";// << "cu.usb*";
    dir.setFilter(QDir::Files|QDir::System);
    dir.setNameFilters(filters);

    QFileInfoList list = dir.entryInfoList();
    for (int i = 0; i < list.size(); ++i)
        ports.push_back(list.at(i).absoluteFilePath());
#endif
    return ports;
}

QString SerialConnection::getPort() const {
    return portname;
}

void SerialConnection::setPort(QString _portname) {
    if (_portname.compare(portname) != 0) {
        portname = _portname;
        close();
        open();
    }
}

void SerialConnection::read() {
    if (!(mode&MODE_DOWNLOAD)) {
        mode |= MODE_DOWNLOAD;
        QMetaObject::invokeMethod(this, "readLoop", Qt::QueuedConnection);
    }
}

void SerialConnection::close() {
    if (context != NULL) {
#ifdef _WIN32
        CloseHandle(*((HANDLE*)context));
        delete (HANDLE*)context;
#else
        ::close(*((int*)context));
        delete (int*)context;
#endif
        context = NULL;
        mode = MODE_NORMAL;
        emit status("disconnected.");
    }
}

void SerialConnection::open() {
    QString status_msg = "connected.";
    QString theport = portname;
    if (context != NULL) {
        close();
    }

    if (theport.length() == 0) {
        QVector<QString> ports = enumerate();
        if (ports.size() > 0) {
            if (ports.size() > 1) theport = ports[1]; else //this line has been added only to use ARK with the light dimmer
            theport = ports[0];
        } else {
            emit status("no serial port found");
            return;
        }
    }

#ifdef _WIN32
    QString comname = QString("\\\\.\\") + theport;
    wchar_t comstr[512];
    memcpy(comstr, comname.toStdWString().c_str(), sizeof(wchar_t)*comname.toStdWString().size());
    HANDLE handle = CreateFile(comstr, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    DCB dcbSerialParams = DCB();

    if (handle == INVALID_HANDLE_VALUE) {
        status_msg = QString("unable to open %1, last error %2.").arg(theport).arg(GetLastError());
    } else if (!GetCommState(handle, &dcbSerialParams)) {
        status_msg = QString("unable to get device attributes");
    } else {
        //Define serial connection parameters for the arduino board
        dcbSerialParams.BaudRate=CBR_38400;
        dcbSerialParams.ByteSize=8;
        dcbSerialParams.StopBits=ONESTOPBIT;
        dcbSerialParams.Parity=NOPARITY;

        //Set the parameters and check for their proper application
        if(!SetCommState(handle, &dcbSerialParams))
            status_msg = QString("unable to set device attributes");
        else {
            context = new HANDLE(handle);
        }
    }
#else
    int fd = ::open(theport.toStdString().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    struct termios toptions;

    if (fd == -1) {
        status_msg = QString("unable to open %1. check if device is already in use.").arg(theport);
    } else if (ioctl(fd, TIOCEXCL) == -1) {
        status_msg = QString("unable to get exclusive access");
    } else if (fcntl(fd, F_SETFL, 0) == -1) {
        status_msg = QString("unable to restore blocking access");
    } else if (tcgetattr(fd, &toptions) < 0) {
        status_msg = QString("unable to get device attributes");
    } else {
        cfsetispeed(&toptions, B38400);
        cfsetospeed(&toptions, B38400);

        // 8N1
        toptions.c_cflag &= ~PARENB;
        toptions.c_cflag &= ~CSTOPB;
        toptions.c_cflag &= ~CSIZE;
        toptions.c_cflag |= CS8;
        // no flow control
        toptions.c_cflag &= ~CRTSCTS;

        //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

        toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
        toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

        toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
        toptions.c_oflag &= ~OPOST; // make raw

        // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
        toptions.c_cc[VMIN]  = 0;
        toptions.c_cc[VTIME] = 0;

        tcsetattr(fd, TCSANOW, &toptions);
        if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
            status_msg = QString("unable to set device attributes");
        } else {
            tcflush(fd, TCIOFLUSH);
            context = new int(fd);
        }
    }
#endif
    emit status(status_msg);
}

void SerialConnection::clearQueue() {
    cmds.clear();
}

void SerialConnection::queueCommand(QByteArray cmd) {
    cmds.push_back(cmd);
    //QMetaObject::invokeMethod(this, "sendQueuedCommand", Qt::QueuedConnection);
//    qDebug() << "Command q'd" << delay.currentTime() << "now on list we have" << cmds.size() << "msgs.";
    emit SendMsgsQueueState(true);
    this->sendQueuedCommand();
}

void SerialConnection::sendQueuedCommand() {
    if (delay.elapsed() < TIMEPERMSG_ms ) {
        if (!this->cmds.isEmpty()) {
            QMetaObject::invokeMethod(this, "sendQueuedCommand", Qt::QueuedConnection);
            //qDebug() << "q func called" << delay.currentTime();
        }
    } else {
        // send next q'd command
        if (!this->cmds.isEmpty()) {
            // send
            delay.restart(); // reset the time counter to count how much to wait before sending another command
            //qDebug() << "Command to send" << delay.currentTime();
            //this->sendCommand(this->cmds[0]);
            QByteArray tmp = !cmds.isEmpty() ? cmds[0] : QByteArray();
            this->sendCommand(tmp);
            //qDebug() << "Command sent" << delay.currentTime();
            !cmds.isEmpty() ? this->cmds.pop_front() : (void) 0;
            //qDebug() << "popped()" << delay.currentTime();
            if (!this->cmds.isEmpty()) {
                QMetaObject::invokeMethod(this, "sendQueuedCommand", Qt::QueuedConnection);
            }
            else {
                emit SendMsgsQueueState(false);
            }
        }
    }
}

void SerialConnection::sendCommand(QByteArray cmd) {
#ifdef _WIN32
    DWORD bytes_sent;
#endif
    mode = MODE_NORMAL;
    if (context != NULL) {
#ifdef _WIN32
        if (!WriteFile(*((HANDLE*)context), cmd.constData(), cmd.length(), &bytes_sent, 0) || bytes_sent != (DWORD)cmd.length())
            emit error(QString("unable to send command."));
#else
        //qDebug() << "Command sending..." << delay.currentTime();
        if (write(*((int*)context), cmd.constData(), cmd.length()) != cmd.length())
            emit error(QString("unable to send command"));
        tcdrain(*((int*)context));
        //qDebug() << "Command sent" << delay.currentTime();
#endif
    } else {
        emit error("cannot send command if disconnected from usb device.");
    }

}

void SerialConnection::resetDelay()
{
    this->delay.restart();
}

void SerialConnection::sendProgram(QString file) {
    if (context != NULL) {
        data.load(file.toStdString());
        page_total = data.size()/PAGE_SIZE+1;
        if (page_total > 220)
            page_total = 220;
        page = page_total;
        if (!(mode&MODE_UPLOAD)) {
            mode |= MODE_UPLOAD;
            delay.start();
            QMetaObject::invokeMethod(this, "programLoop", Qt::QueuedConnection);
        }
    } else {
        emit error("cannot upload program if disconnected from usb device.");
    }
}

void SerialConnection::programLoop() {
#ifdef _WIN32
    DWORD bytes_sent;
#endif
    if (context != NULL && delay.elapsed() > 130) {
        if (page >= page_total) {
            page = 0;
            memset(packet, 0, PACKET_SIZE);
            packet[0] = PACKET_HEADER;
            packet[1] = PACKET_FORWARDMSG;
            packet[2] = page_total;
            packet[11] = BOOTPGM_SIZE;
            packet[PACKET_SIZE-1] = PACKET_HEADER^PACKET_FORWARDMSG^page_total^BOOTPGM_SIZE;
#ifdef _WIN32
            if (!WriteFile(*((HANDLE*)context), packet, PACKET_SIZE, &bytes_sent, 0) || bytes_sent != PACKET_SIZE) {
                mode = MODE_NORMAL;
                emit error(QString("unable to send packet."));
            }
#else
            if (write(*((int*)context), packet, PACKET_SIZE) != PACKET_SIZE) {
                mode = MODE_NORMAL;
                emit error(QString("unable to send packet."));
            }
            tcdrain(*((int*)context));
#endif
        } else {
            packet[0] = PACKET_HEADER;
            packet[1] = PACKET_BOOTPAGE;
            packet[2] = page;
            uint8_t checksum = PACKET_HEADER^PACKET_BOOTPAGE^page;
            uint8_t data_byte;
            for (int i=0; i<PAGE_SIZE; i++) {
                data_byte = data.get(page*PAGE_SIZE+i);
                packet[i+3] = data_byte;
                checksum ^= data_byte;
            }
            packet[PACKET_SIZE-1] = checksum;
#ifdef _WIN32
            if (!WriteFile(*((HANDLE*)context), packet, PACKET_SIZE, &bytes_sent, 0) || bytes_sent != PACKET_SIZE) {
                mode = MODE_NORMAL;
                emit error(QString("unable to send packet."));
            }
            else
                page++;
#else
            if (write(*((int*)context), packet, PACKET_SIZE) != PACKET_SIZE) {
                mode = MODE_NORMAL;
                emit error(QString("unable to send packet."));
            }
            else
                page++;
            tcdrain(*((int*)context));
#endif
        }
        delay.start();
    }
    if (mode&MODE_UPLOAD) {
        QMetaObject::invokeMethod(this, "programLoop", Qt::QueuedConnection);
    }
}

void SerialConnection::readLoop() {
    if (context != NULL) {
#ifdef _WIN32
        DWORD errors;
        COMSTAT status;
        ClearCommError(*((HANDLE*)context), &errors, &status);
        DWORD num = MIN(status.cbInQue, 4096), bytes_read;
        if (num > 0 && ReadFile(*((HANDLE*)context), buf, num, &bytes_read,  NULL)) {
            emit readText(QByteArray((char *)buf, bytes_read));
        }
#else
        int num = ::read(*((int*)context), buf, 4096);
        if (num > 0)
            emit readText(QByteArray((char *)buf, num));
#endif
    }
    if (mode&MODE_DOWNLOAD) {
        QMetaObject::invokeMethod(this, "readLoop", Qt::QueuedConnection);
    }
}
