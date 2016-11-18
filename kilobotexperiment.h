#ifndef KILOBOTEXPERIMENT_H
#define KILOBOTEXPERIMENT_H

#include <QObject>
#include <kilobotenvironment.h>
#include <kilobottracker.h>
#include <kilobotoverheadcontroller.h>

class KilobotExperiment : public QObject
{
    Q_OBJECT
public:
    explicit KilobotExperiment(QObject *parent = 0);

signals:
    void updateKilobotStates();

public slots:
    virtual void initialise() = 0;
    virtual void run() = 0;
//    virtual void createenvironment() = 0;
    virtual void updateKilobotState(Kilobot* kilobot) = 0; // provided in derived class to implement experiment logic for Kilobot state updates

private:
    virtual void updateExperimentRunningState() = 0; // provided in derived class to determine whether experiment should continue running
    KilobotTracker tracker;
    KilobotOverheadController ohc;
    bool experimentRunning;
    QVector <KilobotEnvironment> environments;

};

#endif // KILOBOTEXPERIMENT_H
