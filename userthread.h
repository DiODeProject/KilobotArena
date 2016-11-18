#ifndef USERTHREAD_H
#define USERTHREAD_H

#include <QThread>
#include <QTimer>

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
         connect(kbtracker, SIGNAL(temptemptemp(Kilobot*)), expt, SLOT(updateKilobotState(Kilobot*)));

         this->timer.setInterval(100);
         connect(&this->timer, SIGNAL(timeout()), this->expt, SLOT(run()));
         this->timer.start();

    }

signals:

private:
    USEREXPERIMENT * expt;
    //KilobotTracker * tracker;
    QTimer timer;

    void run()
    {
        QThread::exec();
    }

public slots:
};

#endif // USERTHREAD_H
