#include "mykilobotenvironment.h"
#include <QVector>

#include "kilobot.h"

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent)
{




}

// Only update if environment is dynamic:
void mykilobotenvironment::update()
{


}



//QVector <uint8_t> mykilobotenvironment::getEnvironmentValue(kilobot_pos x, kilobot_pos y)
//{

//    Q_UNUSED(x);
//    Q_UNUSED(y);

//    QVector <uint8_t> retur;

//    retur.resize(9);

//    return retur;

//}

void mykilobotenvironment::generateEnviron(Kilobot* kb)
{


}




