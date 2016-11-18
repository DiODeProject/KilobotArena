#include "mykilobotexperiment.h"
#include "mykilobotenvironment.h"
#include <QDebug>

mykilobotexperiment::mykilobotexperiment(QObject *parent) : KilobotExperiment(parent)
{

}



void mykilobotexperiment::initialise()
{


}

void mykilobotexperiment::run()
{
    emit updateKilobotStates();
}

void mykilobotexperiment::updateKilobotState(Kilobot* kilobot)
{
    qDebug() << "we got a message back!";
}



void mykilobotexperiment::updateExperimentRunningState()
{

}






