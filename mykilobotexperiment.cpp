#include "mykilobotexperiment.h"
#include "mykilobotenvironment.h"
#include <QDebug>

mykilobotexperiment::mykilobotexperiment(QObject *parent) : KilobotExperiment(parent)
{

    // setup the environments here
    environments.push_back(new mykilobotenvironment(/*parameterise?*/));
    environments.push_back(new mykilobotenvironment(/*different parameterisation?*/));

}



void mykilobotexperiment::initialise(bool isResume)
{

    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    } else {
        // probably nothing
    }

}

void mykilobotexperiment::run()
{
    this->time += 0.1; // 100 ms in sec

    if (int(time) % 5 == 0) // every 5s
    {
        emit saveImage(QString("myhappyimage_%1.jpg").arg(int(time), 5,10, QChar('0')));
    }

    // maybe?
    for (int i = 0; i < this->environments.size(); ++i) {
        this->environments[i]->update();
    }

    // required to change kilobot virtual sensory data / environment...
    emit updateKilobotStates();

    if (time > 60.0f) {
      emit experimentComplete();
    }
}

// run once for each kilobot after emitting getInitialKilobotStates() signal
void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobotCopy)
{
    Q_UNUSED(kilobotCopy)

    // setup envs for first time - e.g.:
    if (qrand() % 2) {
      // OMG - just look at this sleek function in the base class that makes this easy as pie!
      this->setCurrentKilobotEnvironment(this->environments[0]);
    } else {
      this->setCurrentKilobotEnvironment(this->environments[1]);
    }
}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy)
{
    // update envs - can be blank if not required
    if (kilobotCopy.getLedColour().r > 200) {
      this->setCurrentKilobotEnvironment(this->environments[0]);
    }

}







