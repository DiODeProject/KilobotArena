#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPoint>
#include "kilobotenvironment.h"

#include <vector>

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    mykilobotenvironment(QObject *parent = 0);

    void generateEnvironment();

    int KBtype = 0;
    int numHome = 0;
    int numGoal = 0;
    int numVS = 2;

//    vector <int> home_locx; // doesn't like me declaring vectors here...
    int home_locx[5];
    int home_locy[5];
    int home_locr[5];
    int goal_locx[5];
    int goal_locy[5];
    int goal_locr[5];
    int goal_s = 1;

    //   QVector <uint8_t> getEnvironmentValue(kilobot_pos, kilobot_pos);

signals:
    void errorMessage(QString);

public slots:
    void update();

private:
    QPoint target;

    void setupGoal(int goal_ind);
    void setupHome(int home_ind);

    //    int environ_setup[3] = {1,0,1}; //# KB types, # goals, # virtual sensors
    //    bool type_setup = 0;
    //    bool goal_setup [2] = {0, 0};
    //    int home_setup [2] = {0, 0};

};
#endif // MYKILOBOTENVIRONMENT_H
