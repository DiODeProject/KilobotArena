#ifndef KILOBOTCALIBRATE_H
#define KILOBOTCALIBRATE_H

#include "kilobotexperiment.h"


class KilobotCalibrate : public KilobotExperiment
{
    Q_OBJECT
public:
    KilobotCalibrate();
    virtual ~KilobotCalibrate() {}


public slots:
        void initialise(bool);
        void run();

private:
        void updateKilobotState(Kilobot kilobotCopy);
        void setupInitialKilobotState(Kilobot kilobotCopy);

};



#endif // TESTLIB_H
