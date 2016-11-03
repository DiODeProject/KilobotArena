#ifndef CALIBWINDOW_H
#define CALIBWINDOW_H

#include <QWidget>
#include <QString>
#include <QSpinBox>

class CalibWindow: public QWidget {
    Q_OBJECT

public:
    CalibWindow(QString title, QWidget *parent = 0);
    void closeEvent(QCloseEvent *);

signals:
    void calibUID(int);
    void calibLeft(int);
    void calibRight(int);
    void calibStraight(int);
    void calibSave();
    void calibStop();

public slots:
    void updateCalib(int);
    void save();

private:
    QSpinBox *values[5];    
};

#endif//CALIBWINDOW_H
