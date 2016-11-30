#include "mykilobotexperiment.h"
#include "mykilobotenvironment.h"
#include <QDebug>

#include <QVector>
#include <QLineF>

//#include "kilobot.h"

//mykilobotenvironment myenviron1;

mykilobotexperiment::mykilobotexperiment(QObject *parent) : KilobotExperiment(parent) {

    // setup the environments here
    environments.push_back(new mykilobotenvironment(/*parameterise?*/));
    environments.push_back(new mykilobotenvironment(/*different parameterisation?*/));

}



void mykilobotexperiment::initialise(bool isResume) {
    // Generate Environments - setup 2 for now:
    setupEnvironment1(); // eventually replace with a loop that generates environments: or // environments[0].generateEnvironment();
    setupEnvironment2(); // eventually replace with // environments[1].generateEnvironment();

    // Initialize Kilobot States:
    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
//        qDebug() << "I am making it here.";
    } else {
        // probably nothing
    }

    // Assign Kilobot IDs here?
//    assignKilobotIDs(mykilobot); // correct this


//    // some random code for now:
//    int num_kbs = 100;
//    qDebug() << num_kbs ;
//    num_kbs = mykilobot.size(); // Size of kilobots
//    qDebug() << num_kbs ;


    // Assign Kilobots to Environments - 1 or 2 at random here for now:
//    setupInitialKilobotEnvironment(mykilobot); // randomly assigns kilobots to 1 of 2 enviroments - correct this

    // Setup Overhead Control:
    myohc.initialiseOHC(); // Does nothing at the moment - correct this

    // Any other setup needed?

}


//
void mykilobotexperiment::run() {
    this->time += 0.1; // 100 ms in sec

    if (int(time) % 5 == 0) // every 5s
    {
        emit saveImage(QString("myhappyimage_%1.jpg").arg(int(time), 5,10, QChar('0')));
    }

    // Update Environments:
    for (int i = 0; i < this->environments.size(); ++i) {
        this->environments[i]->update();
    }


    // Update Kilobot States:
    for (int i = 0; i < this->mykilobot.size(); ++i) {
        //        updateKilobotVS(mykilobot[i]); // update kilobot position and led color
    }
    //    updateKilobotVS(mykilobot[i]);
    //    emit updateKilobotStates();

    // Update Sensor Readings:
    // update virtual sensor reading

    if (time > 60.0f) {
        emit experimentComplete();
    }
}

// Setup the Initial Kilobot Environment:
//   This is run once for each kilobot after emitting getInitialKilobotStates() signal.
//   This assigns kilobots to an environment.
void mykilobotexperiment::setupInitialKilobotEnvironment(Kilobot kilobotCopy) {
    Q_UNUSED(kilobotCopy)

    // setup envs for first time - e.g.:
    // randomly assigns kilobots to either 1 of 2 environments.
    if (qrand() % 2) {
        // OMG - just look at this sleek function in the base class that makes this easy as pie!
        this->setCurrentKilobotEnvironment(this->environments[0]);
    } else {
        this->setCurrentKilobotEnvironment(this->environments[1]);
    }
}

// Update the Kilobots' Environments:
//   This is run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotEnvironment(Kilobot kilobotCopy) {
    // update envs - can be blank if not required
    if (kilobotCopy.getLedColour().r > 200) {
        this->setCurrentKilobotEnvironment(this->environments[0]);
    }

}

// Update Kilobot Virtual Sensor Readings:
//   This is based on current environmental setup:
void mykilobotexperiment::updateKilobotVS(Kilobot kilobotCopy) {

    Q_UNUSED(kilobotCopy)

    // Update the Kilobot virtual sensor readings here. Get kilobot position and led colour.
    // Return bool value for type/s and goal/s and 3-bit value for sensor/s,

    bool type_val = false;
    bool goal_val;


    //    // lalala
    //    QPoint kbPos = QPoint(kilobot.getXPosition(), kilobot.getYPosition());
    //    int dist = QLineF(kbPos, this->target).length();


}

// Assign IDs to Kilobots:
void mykilobotexperiment::assignKilobotIDs(Kilobot kilobotCopy) {

    // assign kilobots IDs....
    Q_UNUSED(kilobotCopy)

}


// Setup Environment 1:
void mykilobotexperiment::setupEnvironment1( ) {
    // For now, do this:
    // For now, do the code below. But eventually replace with this code and/or generateEnvironment which should do this for me:
    //    myenviron1.generateEnvironment(myenviron1.numGoal,myenviron1.numHome, else?);

    myenviron1.goal_s = 1;

    // Replace this whole function which interates with GUI and gets user input on number and location of goals:
    if (myenviron1.numGoal == 1)
    {
        myenviron1.goal_locx[0] = 1100; // define goal 1
        myenviron1.goal_locy[0] = 1100; // define goal 1

        qDebug() << "Env 1 Setup with 1 Goal: Each creates binary VS. Location: (" << myenviron1.goal_locx[0] << "," << myenviron1.goal_locy[0] << ")." ;
    }
    else if (myenviron1.numGoal == 2)
    {
        myenviron1.goal_locx[0] = 1650; // define goal 1
        myenviron1.goal_locy[0] = 1650; // define goal 1
        myenviron1.goal_locx[1] = 550; // define goal 2
        myenviron1.goal_locy[1] = 550; // define goal 2

        qDebug() << "Env 1 Setup with 2 Goals: Each creates binary VS. Location 1: (" << myenviron1.goal_locx[0] << "," << myenviron1.goal_locy[0] << "). Location 2: (" << myenviron1.goal_locx[1] << "," << myenviron1.goal_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 1 Setup with No Goals.";
    }

    if (myenviron1.numHome == 1)
    {
        myenviron1.home_locx[0] = 1100; // define goal 1
        myenviron1.home_locy[0] = 1100; // define goal 1

        qDebug() << "Env 1 Setup with 1 Home: Each creates binary VS. Location: (" << myenviron1.home_locx[0] << "," << myenviron1.home_locy[0] << ")." ;
    }
    else if (myenviron1.numHome == 2)
    {
        myenviron1.home_locx[0] = 1650; // define goal 1
        myenviron1.home_locy[0] = 1650; // define goal 1
        myenviron1.home_locx[1] = 550; // define goal 2
        myenviron1.home_locy[1] = 550; // define goal 2

        qDebug() << "Env 1 Setup with 2 Homes: Each creates binary VS. Location 1: (" << myenviron1.home_locx[0] << "," << myenviron1.home_locy[0] << "). Location 2: (" << myenviron1.home_locx[1] << "," << myenviron1.home_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 1 Setup with No Homes.";
    }

    // Plot the home/goal locations


}

// Setup Environment 2:
void mykilobotexperiment::setupEnvironment2( ) {

    // For now, do the code below. But eventually replace with this code and/or generateEnvironment which should do this for me:
    myenviron2.goal_s = 1;

    // Replace this whole function which interates with GUI and gets user input on number and location of goals:
    if (myenviron2.numGoal == 1)
    {
        myenviron2.goal_locx[0] = 1100; // define goal 1
        myenviron2.goal_locy[0] = 1100; // define goal 1

        qDebug() << "Env 2 Setup with 1 Goal: Each creates binary VS. Location: (" << myenviron2.goal_locx[0] << "," << myenviron2.goal_locy[0] << ")." ;
    }
    else if (myenviron2.numGoal == 2)
    {
        myenviron2.goal_locx[0] = 550; // define goal 1
        myenviron2.goal_locy[0] = 1650; // define goal 1
        myenviron2.goal_locx[1] = 1650; // define goal 2
        myenviron2.goal_locy[1] = 550; // define goal 2

        qDebug() << "Env 2 Setup with 2 Goals: Each creates binary VS. Location 1: (" << myenviron2.goal_locx[0] << "," << myenviron2.goal_locy[0] << "). Location 2: (" << myenviron2.goal_locx[1] << "," << myenviron2.goal_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 2 Setup with No Goals.";
    }

    if (myenviron2.numHome == 1)
    {
        myenviron2.home_locx[0] = 1100; // define goal 1
        myenviron2.home_locy[0] = 1100; // define goal 1

        qDebug() << "Env 2 Setup with 1 Home: Each creates binary VS. Location: (" << myenviron2.home_locx[0] << "," << myenviron2.home_locy[0] << ")." ;
    }
    else if (myenviron2.numHome == 2)
    {
        myenviron2.home_locx[0] = 550; // define goal 1
        myenviron2.home_locy[0] = 1650; // define goal 1
        myenviron2.home_locx[1] = 1650; // define goal 2
        myenviron2.home_locy[1] = 550; // define goal 2

        qDebug() << "Env 2 Setup with 2 Homes: Each creates binary VS. Location 1: (" << myenviron2.home_locx[0] << "," << myenviron2.home_locy[0] << "). Location 2: (" << myenviron2.home_locx[1] << "," << myenviron2.home_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 2 Setup with No Homes.";
    }

}


// ********** GUI Interface Code Here ********** //

// Set the number of Kilobot Types:
void mykilobotexperiment::setKBtype1(int KBtype11) {
    // Setup KB type stuff based on number of KB types
    this->myenviron1.KBtype = KBtype11;
    qDebug() << "Env 1 KB Type set to:" << myenviron1.KBtype ;
}

// Set the number of Goals:
void mykilobotexperiment::setHome1(int numHome11) {
    // Setup Env Features stuff based on number of Env Features
    this->myenviron1.numHome = numHome11;
    qDebug() << "Env 1 # of Home Bases set to:" << myenviron1.numHome ;;
}

// Set the number of Goals:
void mykilobotexperiment::setGoal1(int numGoal11) {
    // Setup Env Features stuff based on number of Env Features
    this->myenviron1.numGoal = numGoal11;
    qDebug() << "Env 1 # of Goals set to:" << myenviron1.numGoal ;
}

// Set the number of Kilobot Types:
void mykilobotexperiment::setKBtype2(int KBtype22) {
    // Setup KB type stuff based on number of KB types
    this->myenviron2.KBtype = KBtype22;
    qDebug() << "Env 2 KB Type set to:" << myenviron2.KBtype ;
}

// Set the number of Goals:
void mykilobotexperiment::setHome2(int numHome22) {
    // Setup Env Features stuff based on number of Env Features
    this->myenviron2.numHome = numHome22;
    qDebug() << "Env 2 # of Home Bases set to:" << myenviron2.numHome;
}

// Set the number of Goals:
void mykilobotexperiment::setGoal2(int numGoal22) {
    // Setup Env Features stuff based on number of Env Features
    this->myenviron2.numGoal = numGoal22;
    qDebug() << "Env 2 # of Goals set to:" << myenviron2.numGoal ;
}

// Set Experiment ID to 1:
void mykilobotexperiment::setExperiment1() {
    // Setup Env Features stuff based on number of Env Features
    this->exptID = 1;
    qDebug() << "Set to Experiment 1.";
}

// Set Experiment ID to 2:
void mykilobotexperiment::setExperiment2() {
    // Setup Env Features stuff based on number of Env Features
    this->exptID = 2;
    qDebug() << "Set to Experiment 2.";
}

// Setup Environment
void mykilobotexperiment::setupExperiment() {

    // Setup Experiment and then initialise everything:
    if (exptID == 1)
    {
        // experiment 1....
        qDebug() << "Experiment 1 is setting up..." ;
    }
    else if (exptID == 2)
    {
        // experiment 2...
        qDebug() << "Experiment 2 is setting up..." ;
    }
    else
    {
        // error
    }

    // Initialize environments, kilobots, OHC, IDs, etc:
    initialise(0);


    //    // Given number of types, goals, and sensors, setup environment:
    //    environ_setup[1] = numKBtype1;
    //    environ_setup[2] = numGoal1;
    //    environ_setup[3] = numHome1;

    //    // Message setup for Kilobots
    ////    int maxbits = 8; // maximum number of bits in a message
    ////    bool message_temp[maxbits] = {true}; // temporary message data
    //    int homebits; // number of bits for each home
    //    int typebits; // number of bits needed for KB types
    //    int goalbits; // number of bits needed for goals

    //    // Set initial types for the environment:
    //    if (environ_setup[1] == 1) {

    //        typebits = 0;

    //        type_setup = 0;  // If it's just 1 environment, set KB to type 1.

    //    } else if (environ_setup[1] == 2) {

    //        typebits = 1;
    //        if (envID1 == 2)
    //        {
    //             type_setup = 1; // If it's environment 2, set KB to type 2.
    //        } else
    //        {
    //             type_setup = 0; // If it's environment 1, set KB to type 1.
    //        }

    //    } else {
    //        typebits = 0;
    //    }

    //    if (environ_setup[2] == 1) {

    //        goalbits = 1;
    //        goal_setup[0] = 1;
    //        setupGoals(environ_setup[2]);

    //    } else if (environ_setup[2] == 2) {

    //        goalbits = 2;

    //        if (envID1 == 2) {
    ////            goal_setup[0] = 1;
    //            goal_setup[1] = 1;
    //            setupGoals(environ_setup[2]);
    //        } else {
    //            goal_setup[0] = 1;
    ////            goal_setup[1] = 1;
    //            setupGoals(environ_setup[2]);
    //        }

    //    } else {
    //        goalbits = 0;
    //        setupGoals(0);
    //    }

    //    if (environ_setup[3] == 1) {

    //        homebits = 3;
    //        home_setup[0] = 1;

    //    } else if (environ_setup[3] == 2) {

    //        homebits = 6;
    //        home_setup[0] = 1;
    //        home_setup[1] = 1;

    //    } else {
    //        homebits = 0;
    //    }

}
