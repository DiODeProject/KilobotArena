#ifndef __SERIALCONN_H__
#define __SERIALCONN_H__

#include <QObject>
#include <QString>
#include <QVector>
#include <QTime>
#include "intelhex.h"

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

public:
    SerialConnection(QObject *p=0, QString="");
    static QVector<QString> enumerate();
    void setPort(QString);
    QString getPort() const;

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

#endif//__SERIALCONN_H__
