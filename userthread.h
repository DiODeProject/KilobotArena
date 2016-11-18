#ifndef USERTHREAD_H
#define USERTHREAD_H

#include <QThread>
#include <QTimer>
#include <QSignalMapper>

#define USEREXPERIMENT mykilobotexperiment //user edit
#include "mykilobotexperiment.h" //user edit

#include "kilobottracker.h"
#include "kilobotoverheadcontroller.h"



class UserThread : public QThread
{
    Q_OBJECT
public:
    explicit UserThread(KilobotTracker * kbtracker, KilobotOverheadController * ohc, QObject *parent = 0)
    {
         expt = new USEREXPERIMENT();

         // join signals / slots
         connect(expt,SIGNAL(updateKilobotStates()), kbtracker, SLOT(updateKilobotStates()));
         connect(expt,SIGNAL(getInitialKilobotStates()), kbtracker, SLOT(getInitialKilobotStates()));

         this->timer.setInterval(100);
         connect(&this->timer, SIGNAL(timeout()), this->expt, SLOT(run()));
         connect(kbtracker, SIGNAL(startExperiment(bool)), this->expt, SLOT(initialise(bool)));
         connect(kbtracker, SIGNAL(startExperiment(bool)), &this->timer, SLOT(start()));
         connect(kbtracker, SIGNAL(stopExperiment()), &this->timer, SLOT(stop()));
         QSignalMapper *mapper = new QSignalMapper(this);
         mapper->setMapping(this->expt, TRACK);
         connect(this->expt, SIGNAL(experimentComplete()), mapper, SLOT(map()));
         connect(mapper, SIGNAL(mapped(int)), kbtracker, SLOT(startStopLoop(int)));

    }
    QTimer timer;

    KilobotExperiment * getExperimentPointer()
    {
        return qobject_cast <KilobotExperiment *> (this->expt);
    }

signals:

private:
    USEREXPERIMENT * expt;

    void run()
    {
        QThread::exec();
    }

public slots:
};

#endif // USERTHREAD_H
