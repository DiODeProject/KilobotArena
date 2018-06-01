#ifndef KILOBOTCALIBRATE_H
#define KILOBOTCALIBRATE_H

#include "kilobotexperiment.h"
#include "kilobotcalibrateenv.h"


class KilobotCalibrate : public KilobotExperiment
{
    Q_OBJECT
public:
    KilobotCalibrate(double kilobot_radius);
    virtual ~KilobotCalibrate() {}
    KilobotCalibrateEnv calibEnvironment;

public slots:
        void initialise(bool);
        void run();
        void drawLineFromEnv(std::vector<cv::Point> pos, QColor col, int thickness, std::string text, bool transparent){
            emit drawLine(pos,col,thickness,text,transparent);
        }

private:
        void updateKilobotState(Kilobot kilobotCopy);
        void setupInitialKilobotState(Kilobot kilobotCopy);

};



#endif // TESTLIB_H
