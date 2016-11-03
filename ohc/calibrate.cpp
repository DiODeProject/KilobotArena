#include <QtGui>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include "calibrate.h"

CalibWindow::CalibWindow(QString title, QWidget *parent): QWidget(parent) {
    QPushButton *save_button = new QPushButton("&Save");
    QPushButton *close_button = new QPushButton("&Close");
    QObject::connect(save_button, SIGNAL(clicked()), this, SLOT(save()));
    QObject::connect(close_button, SIGNAL(clicked()), this, SLOT(close()));

    QVBoxLayout *layout = new QVBoxLayout();
    QHBoxLayout *hbox = new QHBoxLayout();
    QGridLayout *grid = new QGridLayout();
    hbox->addWidget(save_button);
    hbox->addWidget(close_button);
    layout->addLayout(grid);
    layout->addLayout(hbox);
    setLayout(layout);
    setWindowTitle(title);
    setWindowModality(Qt::ApplicationModal);
    setWindowFlags(Qt::WindowStaysOnTopHint|Qt::Dialog);

    QLabel *uid_label = new QLabel("Unique ID");
    QSpinBox *uid_input = new QSpinBox();
    uid_input->setRange(0,65536);
    uid_input->setSingleStep(1);
    QPushButton *uid_button = new QPushButton("Test");

    QLabel *turnleft_label = new QLabel("Turn left");
    QSpinBox *turnleft_input = new QSpinBox();
    turnleft_input->setRange(0,255);
    turnleft_input->setSingleStep(1);
    QPushButton *turnleft_button = new QPushButton("Test");

    QLabel *turnright_label = new QLabel("Turn right");
    QSpinBox *turnright_input = new QSpinBox();
    turnright_input->setRange(0,255);
    turnright_input->setSingleStep(1);
    QPushButton *turnright_button = new QPushButton("Test");

    QLabel *straight_label = new QLabel("Go Straight");
    QSpinBox *straight_input1 = new QSpinBox();
    straight_input1->setRange(0,255);
    straight_input1->setSingleStep(1);
    QSpinBox *straight_input2 = new QSpinBox();
    straight_input2->setRange(0,255);
    straight_input2->setSingleStep(1);
    QHBoxLayout *straight_input = new QHBoxLayout();
    straight_input->addWidget(straight_input1);
    straight_input->addWidget(straight_input2);
    QPushButton *straight_button = new QPushButton("Test");

    grid->addWidget(uid_label, 0, 0);
    grid->addWidget(uid_input, 0, 1);
    grid->addWidget(uid_button, 0, 2);

    grid->addWidget(turnleft_label, 1, 0);
    grid->addWidget(turnleft_input, 1, 1);
    grid->addWidget(turnleft_button, 1, 2);

    grid->addWidget(turnright_label, 2, 0);
    grid->addWidget(turnright_input, 2, 1);
    grid->addWidget(turnright_button, 2, 2);

    grid->addWidget(straight_label, 3, 0);
    grid->addLayout(straight_input, 3, 1);
    grid->addWidget(straight_button, 3, 2);


    QSignalMapper *signalMapper = new QSignalMapper();
    QObject::connect(uid_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
    QObject::connect(turnleft_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
    QObject::connect(turnright_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
    QObject::connect(straight_button, SIGNAL(clicked()), signalMapper, SLOT(map()));

    signalMapper->setMapping(uid_button, 0);
    signalMapper->setMapping(turnleft_button, 1);
    signalMapper->setMapping(turnright_button, 2);
    signalMapper->setMapping(straight_button, 3);

    values[0] = uid_input;
    values[1] = turnleft_input;
    values[2] = turnright_input;
    values[3] = straight_input1;
    values[4] = straight_input2;

    QObject::connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(updateCalib(int)));
}

void CalibWindow::updateCalib(int v) { 
    switch(v) {
        case 0:
            emit calibUID(values[0]->value());
            break;
        case 1:
            emit calibLeft(values[1]->value());
            break;
        case 2:
            emit calibRight(values[2]->value());
            break;
        case 3:
            emit calibStraight(values[3]->value() | ((values[4]->value())&0xFF)<<8);
            break;
        default:
            break;
    }
}

void CalibWindow::closeEvent(QCloseEvent *event) {
    emit calibStop();
    event->accept();
}

void CalibWindow::save() {
    emit calibSave();
}
