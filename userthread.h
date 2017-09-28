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

         // add the kilobot ID assignment experiment
         this->expts.push_back(new KilobotIDAssignment);
         this->currExpt = expts.size()-1;
         this->connectExpt(this->currExpt);

    }
    QTimer timer;

    KilobotExperiment * getExperimentPointer()
    {
        if (this->expts.size() > currExpt) {
            return this->expts[currExpt];
        } else {
            return NULL;
        }
    }

signals:
    void setLibName(QString);
    void setGUILayout(QWidget *);

private:
    QVector < KilobotExperiment * > expts;
    KilobotOverheadController * ohc;
    KilobotTracker * kbtracker;

    QString currExptFilename;

    int currExpt = 0;

    void run()
    {
        QThread::exec();
    }

    QSignalMapper * mapper = NULL;


public slots:
        void connectExpt(int num) {
            if (num < this->expts.size()) {
                this->currExpt = num;

                // join signals / slots

                // expt -> tracker
                connect(expts[currExpt],SIGNAL(updateKilobotStates()), kbtracker, SLOT(updateKilobotStates()));
                connect(expts[currExpt],SIGNAL(getInitialKilobotStates()), kbtracker, SLOT(getInitialKilobotStates()));
                connect(expts[currExpt],SIGNAL(setTrackingType(int)), kbtracker, SLOT(setTrackingType(int)));

                if (mapper != NULL) mapper->deleteLater();
                mapper = new QSignalMapper(this);
                mapper->setMapping(this->expts[currExpt], TRACK);
                connect(this->expts[currExpt], SIGNAL(experimentComplete()), mapper, SLOT(map()));
                connect(mapper, SIGNAL(mapped(int)), kbtracker, SLOT(LOOPstartstop(int)));

                // drawing signal / slots
                connect(expts[currExpt], SIGNAL(drawCircle(QPointF,float,QColor,int, std::string)), this->kbtracker, SLOT(drawCircle(QPointF,float,QColor,int, std::string)));
                connect(expts[currExpt], SIGNAL(clearDrawings()), this->kbtracker, SLOT(clearDrawings()));
                connect(expts[currExpt], SIGNAL(drawCircleOnRecordedImage(QPointF,float,QColor,int,std::string)), this->kbtracker, SLOT(drawCircleOnRecordedImage(QPointF,float,QColor,int,std::string)));
                connect(expts[currExpt], SIGNAL(clearDrawingsOnRecordedImage()), this->kbtracker, SLOT(clearDrawingsOnRecordedImage()));

                // save image/video
                connect(expts[currExpt], SIGNAL(saveImage(QString)), this->kbtracker, SLOT(saveImage(QString)));
                connect(expts[currExpt], SIGNAL(saveVideoFrames(QString,unsigned int)), this->kbtracker, SLOT(saveVideoFrames(QString,unsigned int)));


                // clock for experiment
                this->timer.setInterval(expts[currExpt]->serviceInterval);
                connect(&this->timer, SIGNAL(timeout()), this->expts[currExpt], SLOT(run()));

                // tracker outputs
                connect(kbtracker, SIGNAL(startExperiment(bool)), this->expts[currExpt], SLOT(initialise(bool)));
                connect(kbtracker, SIGNAL(startExperiment(bool)), &this->timer, SLOT(start()));
                connect(kbtracker, SIGNAL(stopExperiment()), this->expts[currExpt], SLOT(stopExperiment()));
                connect(kbtracker, SIGNAL(stopExperiment()), &this->timer, SLOT(stop()));

                // ohc
                connect(this->expts[currExpt], SIGNAL(broadcastMessage(kilobot_broadcast)), this->ohc, SLOT(broadcastMessage(kilobot_broadcast)));
                connect(this->expts[currExpt], SIGNAL(signalKilobot(kilobot_message)), this->ohc, SLOT(signalKilobot(kilobot_message)));

                this->kbtracker->expt = this->expts[currExpt];

            }
        }

        void chooseInternalExperiments(int num) {
            this->currExptFilename = "";
            if (num < 3) {
                while (this->expts.size() > 0) {
                    this->expts[0]->deleteLater();
                    this->expts.pop_front();
                }
                if (num == 0) { // assign IDs                           
                    this->expts.push_back(new KilobotIDAssignment(BINARY));
                    this->currExpt = expts.size()-1;
                    this->connectExpt(this->currExpt);
                }

                if (num == 1) { // assign IDs
                    this->expts.push_back(new KilobotIDAssignment(BASETHREE));
                    this->currExpt = expts.size()-1;
                    this->connectExpt(this->currExpt);
                }

                if (num == 2) { // calibrate
                    this->expts.push_back(new KilobotCalibrate);
                    this->currExpt = expts.size()-1;
                    this->connectExpt(this->currExpt);
                }
            }
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

                while (this->expts.size() > 0) {
                    this->expts[0]->deleteLater();
                    this->expts.pop_front();
                }

                expts.push_back(createExperiment());
                emit setLibName(filename);
                // set as current experiment for now
                this->currExpt = expts.size()-1;
                this->connectExpt(this->currExpt);

                // setup GUI
                emit setGUILayout(expts[this->currExpt]->createGUI());


            } else {
                emit setLibName(QString("<load failed: ") + library.errorString() + QString(">"));
                this->expts.clear();
            }
        }

        bool exptLoaded() {return this->expts.size() != 0;}
};

#endif // USERTHREAD_H
