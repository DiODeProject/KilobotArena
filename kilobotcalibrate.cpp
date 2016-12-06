#include "kilobotcalibrate.h"
#include "kilobotcalibrateenv.h"
#include <QDebug>
#include <QThread>

KilobotCalibrate::KilobotCalibrate()
{
    // setup the environments here
    this->environments.push_back(new KilobotCalibrateEnv());

    // link to OHC
    for (int i = 0; i < this->environments.size(); ++i) {
        connect(this->environments[i],SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));
    }

    this->serviceInterval = 250;

}



void KilobotCalibrate::initialise(bool isResume)
{

    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    }

    emit setTrackingType(POS | ROT | ADAPTIVE_LED);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

}

void KilobotCalibrate::run()
{
    this->time += 0.1; // 100 ms in sec

    // maybe?
    for (int i = 0; i < this->environments.size(); ++i) {
        this->environments[i]->update();
    }

    // required to change kilobot virtual sensory data / environment...
    emit clearDrawings();
    emit updateKilobotStates();

    if (((KilobotCalibrateEnv *)this->environments[0])->rotDone) {
        ((KilobotCalibrateEnv *)this->environments[0])->rotDone = false;
        // broadcast to store calib data
        kilobot_broadcast msg;
        msg.type = 110;
        msg.data = 0;
        emit broadcastMessage(msg);
    }
    if (((KilobotCalibrateEnv *)this->environments[0])->strDone) {
        ((KilobotCalibrateEnv *)this->environments[0])->strDone = false;
        // broadcast to store calib data
        kilobot_broadcast msg;
        msg.type = 111;
        msg.data = 0;
        emit broadcastMessage(msg);
    }

}

// run once for each kilobot after emitting getInitialKilobotStates() signal
void KilobotCalibrate::setupInitialKilobotState(Kilobot /*kilobotCopy*/)
{

    this->setCurrentKilobotEnvironment(this->environments[0]);



}

// run once for each kilobot after emitting updateKilobotStates() signal
void KilobotCalibrate::updateKilobotState(Kilobot kilobotCopy)
{

    emit drawCircle(kilobotCopy.getPosition(), 20, QColor(255,0,0));

}
