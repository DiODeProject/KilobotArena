#include "mykilobotenvironment.h"
#include <QVector>
#include <QLineF>

#include "kilobot.h"

#include <QDebug>

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent)
{

    // hard-code for now
    this->target = QPoint(1100,1100); // millimetres


}

// Only update if environment is dynamic:
void mykilobotenvironment::update()
{


}

// Called by "initialise" in myKBexperiment and will be run once at start to generate environmental definitions.
void mykilobotenvironment::generateEnvironment()
{

    // Define Environment:

    // call any functions to setup features in the environment (goals, homes locations and parameters).


    // Don't do this anymore.... do something else/code below:
//    setEnvironment1( );
//    setEnvironment2( );




    //    // Setup goal environmental features:
    //    setupGoal(numGoal); // should connect with UI to setup goal locations and parameters

    //    // Setup home base features:
    //    setupHome(numHome);  // should connect with UI to setup home locations

}


void mykilobotenvironment::setupGoal(int goal_ind)
{

    // Replace with something that is GUi-friendly and can be added by user with a click for the loc?

//    qDebug() << "Setup Goals" ;

//    emit errorMessage("Setup Goals");

}

void mykilobotenvironment::setupHome(int home_ind)
{

    // Replace with something that is GUi-friendly and can be added by user with a click for the loc?

//    qDebug() << "Setup Home Bases" ;

//    emit errorMessage("Setup Home Bases");

}
