#ifndef SERIALWINDOW_H
#define SERIALWINDOW_H

#include <QWidget>
#include <QString>

class QTextEdit;

class SerialWindow: public QWidget {
    Q_OBJECT

public:
    SerialWindow(QString title, QWidget *parent = 0);

public slots:
    void addText(QString);
    void clear();

private:
    QTextEdit *text_edit;
};

#endif//SERIALWINDOW_H
