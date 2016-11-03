#ifndef __FTDICONN_H__
#define __FTDICONN_H__

#include <ftdi.h>
#include <QObject>
#include <QString>
#include <QTime>
#include "intelhex.h"

class FTDIConnection: public QObject {
    Q_OBJECT

private:
    struct ftdi_context *ftdic;
    intelhex::hex_data data;
    QTime delay;
    int mode;
    int page;
    int page_total;

public:
    FTDIConnection(QObject *p=0);

signals:
    void readText(QString);
    void status(QString);
    void error(QString);

public slots:
    void sendCommand(QByteArray);
    void sendProgram(QString);
    void open();
    void close();
    void read();

private slots:
    void readLoop();
    void programLoop();
};

#endif//__FTDICONN_H__
