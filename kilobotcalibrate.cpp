#include "kilobotcalibrate.h"
#include "kilobotcalibrateenv.h"
#include <QDebug>
#include <QThread>

KilobotCalibrate::KilobotCalibrate(double kilobot_radius)
{
    // setup the environments here
    calibEnvironment.setKilobotRadius(kilobot_radius);

    // link to OHC
    connect(&calibEnvironment,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));
    connect(&calibEnvironment,SIGNAL(drawLine(std::vector<cv::Point>,QColor,int,std::string,bool)), this, SLOT(drawLineFromEnv(std::vector<cv::Point>,QColor,int,std::string,bool)));

    this->serviceInterval = 100;
}



void KilobotCalibrate::initialise(bool isResume)
{

    emit clearDrawings();

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

    this->calibEnvironment.update();

    // required to change kilobot virtual sensory data / environment...
    emit clearDrawings();
    emit updateKilobotStates();

    if (calibEnvironment.rotDone) {
        qDebug() << "Calibration of right/left rotation completed.";
        calibEnvironment.rotDone = false;
        // broadcast to store calib data
        kilobot_broadcast msg;
        msg.type = 110;
        emit broadcastMessage(msg);
    }
    if (calibEnvironment.strDone) {
        qDebug() << "Calibration of straight motion completed.";
        calibEnvironment.strDone = false;
        // broadcast to store calib data
        kilobot_broadcast msg;
        msg.type = 111;
        emit broadcastMessage(msg);
    }
}

// run once for each kilobot after emitting getInitialKilobotStates() signal
void KilobotCalibrate::setupInitialKilobotState(Kilobot /*kilobotCopy*/)
{
    this->setCurrentKilobotEnvironment(&this->calibEnvironment);
}

// run once for each kilobot after emitting updateKilobotStates() signal
void KilobotCalibrate::updateKilobotState(Kilobot kilobotCopy)
{

}
