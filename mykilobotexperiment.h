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
    void initialise(bool);
    void run();

private:
    void updateKilobotState(Kilobot kilobotCopy);
    void setupInitialKilobotState(Kilobot kilobotCopy);

};

#endif // MYKILOBOTEXPERIMENT_H
