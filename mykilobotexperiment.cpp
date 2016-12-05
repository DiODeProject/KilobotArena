#include "mykilobotexperiment.h"
#include "mykilobotenvironment.h"

#include "math.h"

#include <QDebug>
#include <QVector>
#include <QLineF>

//#include "kilobot.h"
#include <QPainter>


using namespace cv;
using namespace std;

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
    // Update Kilobot States:
    for (int i = 0; i < this->mykilobot.size(); ++i) {
//        setupInitialKilobotEnvironment(mykilobot[i]); // randomly assigns kilobots to 1 of 2 enviroments - correct this
//        updateKilobotVS(mykilobot[i]);
    }

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
    //    // update envs - can be blank if not required
    //    if (kilobotCopy.getLedColour().r > 200) {
    //        this->setCurrentKilobotEnvironment(this->environments[0]);
    //    }

}

// Update Kilobot Virtual Sensor Readings:
//   This is based on current environmental setup:
void mykilobotexperiment::updateKilobotVS(Kilobot kilobotCopy) {

    //    Q_UNUSED(kilobotCopy)

    // Update the Kilobot virtual sensor readings here. Get kilobot position and led colour.
    QPoint kbPos = QPoint(kilobotCopy.getXPosition(), kilobotCopy.getYPosition());
    //    int dist = QLineF(kbPos, this->target).length();

    // Convert sensor value to a virtual sensor reading between 1 and 128:
    int minDist = 0; // distance in mm
    int maxDist = 3000; // distance in mm

    double kbDist;
    kbDist = sqrt((kbPos.x())^2 + (kbPos.x())^2); // Should be distance from the kilobot's environment goals.

    double kbVSval;
    kbVSval = 127*kbDist/(maxDist-minDist);

    int finVSval;
    finVSval = (int)round(kbVSval);

    kilobotCopy.vsensorValue = finVSval;

    int xval = finVSval;

    vector<int> ret;
    while(xval) {
      if (xval&1)
        ret.push_back(1);
      else
        ret.push_back(0);
      xval>>=1;
    }
    reverse(ret.begin(),ret.end());
//    return ret;

    qDebug() << ret ;

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
        myenviron1.goal_locx[0] = 150; // define goal 1
        myenviron1.goal_locy[0] = 450; // define goal 1
        myenviron1.goal_locr[0] = 50;

        qDebug() << "Env 1 Setup with 1 Goal: Each creates binary VS. Location: (" << myenviron1.goal_locx[0] << "," << myenviron1.goal_locy[0] << ")." ;
    }
    else if (myenviron1.numGoal == 2)
    {
        myenviron1.goal_locx[0] = 150; // define goal 1
        myenviron1.goal_locy[0] = 450; // define goal 1
        myenviron1.goal_locr[0] = 50;

        myenviron1.goal_locx[1] = 450; // define goal 2
        myenviron1.goal_locy[1] = 150; // define goal 2
        myenviron1.goal_locr[1] = 50;

        qDebug() << "Env 1 Setup with 2 Goals: Each creates binary VS. Location 1: (" << myenviron1.goal_locx[0] << "," << myenviron1.goal_locy[0] << "). Location 2: (" << myenviron1.goal_locx[1] << "," << myenviron1.goal_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 1 Setup with No Goals.";
    }

    if (myenviron1.numHome == 1)
    {
        myenviron1.home_locx[0] = 150; // define goal 1
        myenviron1.home_locy[0] = 450; // define goal 1
        myenviron1.home_locr[0] = 50;

        qDebug() << "Env 1 Setup with 1 Home: Each creates binary VS. Location: (" << myenviron1.home_locx[0] << "," << myenviron1.home_locy[0] << ")." ;
    }
    else if (myenviron1.numHome == 2)
    {
        myenviron1.home_locx[0] = 150; // define goal 1
        myenviron1.home_locy[0] = 450; // define goal 1
        myenviron1.home_locr[0] = 50;

        myenviron1.home_locx[1] = 450; // define goal 2
        myenviron1.home_locy[1] = 150; // define goal 2
        myenviron1.home_locr[1] = 50;

        qDebug() << "Env 1 Setup with 2 Homes: Each creates binary VS. Location 1: (" << myenviron1.home_locx[0] << "," << myenviron1.home_locy[0] << "). Location 2: (" << myenviron1.home_locx[1] << "," << myenviron1.home_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 1 Setup with No Homes.";
    }

    //    plotEnvironment(&myenviron1);
}

// Setup Environment 2:
void mykilobotexperiment::setupEnvironment2( ) {

    // For now, do the code below. But eventually replace with this code and/or generateEnvironment which should do this for me:
    myenviron2.goal_s = 1;

    // Replace this whole function which interates with GUI and gets user input on number and location of goals:
    if (myenviron2.numGoal == 1)
    {
        myenviron2.goal_locx[0] = 150; // define goal 1
        myenviron2.goal_locy[0] = 150; // define goal 1
        myenviron2.goal_locr[0] = 50;

        qDebug() << "Env 2 Setup with 1 Goal: Each creates binary VS. Location: (" << myenviron2.goal_locx[0] << "," << myenviron2.goal_locy[0] << ")." ;
    }
    else if (myenviron2.numGoal == 2)
    {
        myenviron2.goal_locx[0] = 150; // define goal 1
        myenviron2.goal_locy[0] = 150; // define goal 1
        myenviron2.goal_locr[0] = 50;

        myenviron2.goal_locx[1] = 450; // define goal 2
        myenviron2.goal_locy[1] = 450; // define goal 2
        myenviron2.goal_locr[1] = 50;

        qDebug() << "Env 2 Setup with 2 Goals: Each creates binary VS. Location 1: (" << myenviron2.goal_locx[0] << "," << myenviron2.goal_locy[0] << "). Location 2: (" << myenviron2.goal_locx[1] << "," << myenviron2.goal_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 2 Setup with No Goals.";
    }

    if (myenviron2.numHome == 1)
    {
        myenviron2.home_locx[0] = 150; // define goal 1
        myenviron2.home_locy[0] = 150; // define goal 1
        myenviron2.home_locr[0] = 50;

        qDebug() << "Env 2 Setup with 1 Home: Each creates binary VS. Location: (" << myenviron2.home_locx[0] << "," << myenviron2.home_locy[0] << ")." ;
    }
    else if (myenviron2.numHome == 2)
    {
        myenviron2.home_locx[0] = 150; // define goal 1
        myenviron2.home_locy[0] = 150; // define goal 1
        myenviron2.home_locr[0] = 50;

        myenviron2.home_locx[1] = 450; // define goal 2
        myenviron2.home_locy[1] = 450; // define goal 2
        myenviron2.home_locr[1] = 50;

        qDebug() << "Env 2 Setup with 2 Homes: Each creates binary VS. Location 1: (" << myenviron2.home_locx[0] << "," << myenviron2.home_locy[0] << "). Location 2: (" << myenviron2.home_locx[1] << "," << myenviron2.home_locy[1] << ")." ;
    }
    else
    {
        qDebug() << "Env 2 Setup with No Homes.";
    }

    plotEnvironment(&myenviron2);

}

// Plot Environment on frame:
void mykilobotexperiment::plotEnvironment(mykilobotenvironment *myenvironcopy) {

    // Plot the home/goal locations
    String imagefilename( "/home/chelseas/Pictures/kilobot_arena.jpg" ); // by default
    finalImage = imread( imagefilename, IMREAD_COLOR ); // load something into finalimage of correct size

    // Check for Valid Input:
    if (this->finalImage.empty()){
        qDebug() << "Empty Image Error";
        return;
    }

    Mat res2;
    Mat display;
    this->finalImage.copyTo(display);
    res2 = this->finalImage;

    // the *2 is an assumption - should always be true...
    //        cv::cvtColor(display, display, CV_GRAY2RGB);

    cv::resize(display,display,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));

    // convert to C header for easier mem ptr addressing
    IplImage imageIpl = display;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);


    // Copy Pixmap and draw environment:
    //    qDebug() << pix.size() ;
    QPixmap pix1(pix.size());
    pix1 = pix;

    QPainter painter(&pix1);
    QPen Red((QColor(255,50,100)),5);
    QPen Green((QColor(100,255,50)),5);
    QPen Blue((QColor(100,50,255)),5);

    // Other random drawing stuff for now:
    //    painter.setPen(Red);
    //    painter.drawLine(50,50,500,50);
    //    painter.setPen(Blue);
    //    painter.drawLine(200,200,500,200);
    //    painter.setPen(Green);
    //    painter.drawLine(300,400,500,400);

    //    painter.setPen(Qt::blue);
    //    painter.setFont(QFont("Arial", 30));
    //    painter.drawText(rect(), Qt::AlignCenter, "Qt");

    // Setup Circles to Draw:
    vector<vector<int>> env_circles1(10, vector<int>(3));
    vector<vector<int>> env_circles2(10, vector<int>(3));

    //    // Copy circles from environment 1:
    //    //    env_circles[0] = {300,300,100};
    //    int num_circles = myenvironcopy->numGoal;
    //    //    int num_circles = myenvironcopy->numGoal + myenvironcopy->numHome;
    //    for (int i = 0; i < myenvironcopy->numGoal; i++){
    //        env_circles[i] = {myenvironcopy->goal_locx[i],myenvironcopy->goal_locy[i],myenvironcopy->goal_locr[i]};
    //    }
    //    //    for (int i = 0; i < myenvironcopy->numHome; i++){
    //    //        env_circles[i] = {myenvironcopy->home_locx[i],myenvironcopy->home_locy[i],myenvironcopy->home_locr[i]};
    //    //    }

    //    qDebug() << "Found" <<  num_circles << "circles";

    //    // Draw Circles:
    //    for( int i = 0; i < num_circles; i++ ) {
    //        painter.setPen(Blue);
    //        painter.drawEllipse(env_circles[i][0],env_circles[i][1],env_circles[i][2],env_circles[i][2]);
    //    }



    // Copy circles from environment 1:
    //    env_circles[0] = {300,300,100};
    int num_circles1 = myenviron1.numGoal;
    //    int num_circles = myenvironcopy->numGoal + myenvironcopy->numHome;
    for (int i = 0; i < myenviron1.numGoal; i++){
        env_circles1[i] = {myenviron1.goal_locx[i],myenviron1.goal_locy[i],myenviron1.goal_locr[i]};
    }

    // Draw Circles for Environment 1:
    for( int i = 0; i < num_circles1; i++ ) {
        painter.setPen(Blue);
        painter.drawEllipse(env_circles1[i][0],env_circles1[i][1],env_circles1[i][2],env_circles1[i][2]);
    }

    // Copy circles from environment 2:
    //    env_circles[0] = {300,300,100};
    int num_circles2 = myenviron2.numGoal;
    //    int num_circles = myenvironcopy->numGoal + myenvironcopy->numHome;
    for (int i = 0; i < myenviron2.numGoal; i++){
        env_circles2[i] = {myenviron2.goal_locx[i],myenviron2.goal_locy[i],myenviron2.goal_locr[i]};
    }

    // Draw Circles for Environment 2:
    for( int i = 0; i < num_circles2; i++ ) {
        painter.setPen(Red);
        painter.drawEllipse(env_circles2[i][0],env_circles2[i][1],env_circles2[i][2],env_circles2[i][2]);
    }

    qDebug() << "I drew circles!";

    setExptImage(pix1);
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
}
