#ifndef __SERIALCONN_H__
#define __SERIALCONN_H__

#include <QObject>
#include <QString>
#include <QVector>
#include <QTime>
#include "intelhex.h"

#define TIMEPERMSG_ms 50

class SerialConnection: public QObject {
    Q_OBJECT

private:
    QString portname;
    void *context;
    intelhex::hex_data data;
    QTime delay;
    int mode;
    int page;
    int page_total;
    QVector < QByteArray > cmds;

public:
    SerialConnection(QObject *p=0, QString="");
    static QVector<QString> enumerate();
    void setPort(QString);
    QString getPort() const;
    void clearQueue();
    int cmdQueueSize() {return this->cmds.size();}

signals:
    void readText(QString);
    void status(QString);
    void error(QString);
    void SendMsgsQueueState(bool);

public slots:
    void queueCommand(QByteArray);
    void sendQueuedCommand();
    void sendCommand(QByteArray);
    void sendProgram(QString);
    void open();
    void close();
    void read();
    void resetDelay();

private slots:
    void readLoop();
    void programLoop();
};

#endif//__SERIALCONN_H__
