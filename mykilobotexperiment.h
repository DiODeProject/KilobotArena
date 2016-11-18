#ifndef MYKILOBOTEXPERIMENT_H
#define MYKILOBOTEXPERIMENT_H

#include <QObject>
#include "kilobotexperiment.h"

class mykilobotexperiment : public KilobotExperiment
{
    Q_OBJECT
public:
    explicit mykilobotexperiment(QObject *parent = 0);

public slots:
    void initialise();
    void run();
//    void getenvironments();
    void updateKilobotState(Kilobot* kilobot); // provided in derived class to implement experiment logic for Kilobot state updates


private:
    void updateExperimentRunningState(); // provided in derived class to determine whether experiment should continue running
    KilobotTracker tracker;
    KilobotOverheadController ohc;
    bool experimentRunning;
    QVector <KilobotEnvironment> environments;


};

#endif // MYKILOBOTEXPERIMENT_H
