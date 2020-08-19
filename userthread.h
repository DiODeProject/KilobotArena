#ifndef USERTHREAD_H
#define USERTHREAD_H

#include <QThread>
#include <QTimer>
#include <QSignalMapper>
#include <QLibrary>

//#define USEREXPERIMENT mykilobotexperiment //user edit
//#include "mykilobotexperiment.h" //user edit
#include "kilobotexperiment.h"
#include "kilobotidassignment.h"
#include "kilobotcalibrate.h"

#include "kilobottracker.h"
#include "kilobotoverheadcontroller.h"
#include <QDebug>


class UserThread : public QThread
{
    Q_OBJECT
public:
    explicit UserThread(KilobotTracker * kbtracker, KilobotOverheadController * ohc, QObject *parent = 0)
    {

        Q_UNUSED(parent)

         this->ohc = ohc;
         this->kbtracker = kbtracker;

         // set the kilobot ID assignment as current experiment
         this->currExpt = new KilobotIDAssignment(BINARY);
         this->connectExpt(ID_ASSIGNMENT);

    }
    QTimer timer;

    KilobotExperiment * getExperimentPointer() { return currExpt; }

signals:
    void setLibName(QString);
    void setGUILayout(QWidget *);

private:
    KilobotOverheadController * ohc;
    KilobotTracker * kbtracker;

    QString currExptFilename;

    KilobotExperiment * currExpt;
    experimentType currExptType;

    void run()
    {
        QThread::exec();
    }

    QSignalMapper * mapper = NULL;


public slots:
        void connectExpt(int expType) {
            this->currExptType = (experimentType) expType;

            // join signals / slots

            // expt -> tracker
            connect(this->currExpt,SIGNAL(updateKilobotStates()), kbtracker, SLOT(updateKilobotStates()));
            connect(this->currExpt,SIGNAL(getInitialKilobotStates()), kbtracker, SLOT(getInitialKilobotStates()));
            connect(this->currExpt,SIGNAL(setTrackingType(int)), kbtracker, SLOT(setTrackingType(int)));
            connect(this->currExpt,SIGNAL(sendBroadcastingState(bool)), kbtracker, SLOT(updateExperimentBroadcastingState(bool)));

            if (mapper != NULL) mapper->deleteLater();
            mapper = new QSignalMapper(this);
            mapper->setMapping(this->currExpt, this->currExptType);
            connect(this->currExpt, SIGNAL(experimentComplete()), mapper, SLOT(map()));
            connect(mapper, SIGNAL(mapped(int)), kbtracker, SLOT(LOOPstartstop(int)));

            // drawing signal / slots
            connect(this->currExpt, SIGNAL(drawCircle(QPointF,float,QColor,int, std::string, bool)), this->kbtracker, SLOT(drawCircle(QPointF,float,QColor,int, std::string, bool)));
            connect(this->currExpt, SIGNAL(drawLine(std::vector<cv::Point>,QColor,int,std::string,bool)), this->kbtracker, SLOT(drawLine(std::vector<cv::Point>,QColor,int,std::string,bool)));
            connect(this->currExpt, SIGNAL(clearDrawings()), this->kbtracker, SLOT(clearDrawings()));
            connect(this->currExpt, SIGNAL(drawCircleOnRecordedImage(QPointF,float,QColor,int,std::string)), this->kbtracker, SLOT(drawCircleOnRecordedImage(QPointF,float,QColor,int,std::string)));
            connect(this->currExpt, SIGNAL(clearDrawingsOnRecordedImage()), this->kbtracker, SLOT(clearDrawingsOnRecordedImage()));

            // save image/video
            connect(this->currExpt, SIGNAL(saveImage(QString)), this->kbtracker, SLOT(saveImage(QString)));
            connect(this->currExpt, SIGNAL(saveVideoFrames(QString,unsigned int)), this->kbtracker, SLOT(saveVideoFrames(QString,unsigned int)));


            // clock for experiment
            this->timer.setInterval(this->currExpt->serviceInterval);
            connect(&this->timer, SIGNAL(timeout()), this->currExpt, SLOT(run()));

            // tracker outputs
            connect(kbtracker, SIGNAL(startExperiment(bool)), this->currExpt, SLOT(initialise(bool)));
            connect(kbtracker, SIGNAL(startExperiment(bool)), &this->timer, SLOT(start()));
            connect(kbtracker, SIGNAL(stopExperiment()), this->currExpt, SLOT(stopExperiment()));
            connect(kbtracker, SIGNAL(stopExperiment()), &this->timer, SLOT(stop()));
            connect(kbtracker, SIGNAL(setRuntimeIdentificationLock(bool)), this->currExpt, SLOT(setRuntimeIdentificationLock(bool)));

            // ohc
            connect(this->currExpt, SIGNAL(broadcastMessage(kilobot_broadcast)), this->ohc, SLOT(broadcastMessage(kilobot_broadcast)));
            connect(this->currExpt, SIGNAL(signalKilobot(kilobot_message)), this->ohc, SLOT(signalKilobot(kilobot_message)));
            connect(this->ohc, SIGNAL(SignalMsgsQueueState(bool)), this->currExpt, SLOT(GetMsgsQueueState(bool)));

            this->kbtracker->expt = this->currExpt;

        }

        void chooseInternalExperiments(experimentType expType, int opt=0) {
            this->currExptFilename = "";
            switch (expType) {
            case ID_ASSIGNMENT:
                if (opt==2){
                    if (exptLoaded()) this->currExpt->deleteLater();
                    this->currExpt = new KilobotIDAssignment(BINARY);
                }
                if (opt==3){
                    if (exptLoaded()) this->currExpt->deleteLater();
                    this->currExpt = new KilobotIDAssignment(BASETHREE);
                }
                break;
            case CALIBRATION:
                if (exptLoaded()) this->currExpt->deleteLater();
                this->currExpt = new KilobotCalibrate( (this->kbtracker->kbMinSize+this->kbtracker->kbMaxSize)/2.0 );
                break;
            default:
                break;
            }
            this->connectExpt(expType);
        }

        void loadLibrary(QString filename) {

            // load library
            QLibrary library(filename);

            // resolve access to class
            typedef KilobotExperiment* (*createExperimentFunction)();
            createExperimentFunction createExperiment = (createExperimentFunction)library.resolve("createExpt");

            if (createExperiment) {

                if (currExptFilename != filename) {
                    currExptFilename = filename;
                }

                if (exptLoaded()) this->currExpt->deleteLater();
                this->currExpt = createExperiment();
                emit setLibName(filename);
                // set as current experiment for now
                this->connectExpt(USER_EXP);

                // setup GUI
                emit setGUILayout(this->currExpt->createGUI());


            } else {
                emit setLibName(QString("<load failed: ") + library.errorString() + QString(">"));
                this->currExpt = NULL;
            }
        }

        bool exptLoaded() {return this->currExpt != NULL;}
};

#endif // USERTHREAD_H
