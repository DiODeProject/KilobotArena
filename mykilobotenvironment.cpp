#include "mykilobotenvironment.h"
#include <QVector>
#include <QLineF>

#include "kilobot.h"

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent)
{

    // hard-code for now
    this->target = QPoint(1100,1100); // millimetres


}

// Only update if environment is dynamic:
void mykilobotenvironment::update()
{


}


void mykilobotenvironment::generateEnvironment(Kilobot kilobot)
{
    // lalala
    QPoint kbPos = QPoint(kilobot.getXPosition(), kilobot.getYPosition());
    int dist = QLineF(kbPos, this->target).length();

    // construct data packet of 10 bits

}




