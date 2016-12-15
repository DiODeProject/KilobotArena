#include "kilobotcalibrateenv.h"
#include <QVector>
#include <QLineF>
#include <QDebug>



#include "kilobot.h"

KilobotCalibrateEnv::KilobotCalibrateEnv(QObject *parent) : KilobotEnvironment(parent)
{

    qDebug() << "Calibrate selected";
    qDebug() << "Giovanni's version";

}

// Only update if environment is dynamic:
void KilobotCalibrateEnv::update()
{


}

bool KilobotCalibrateEnv::evaluateSpeed(int step, int timeInterval, double robotRadius, int &speed, int index){
    QLineF line = QLineF(this->posLog[index][step], this->posLog[index].back());
    double distance = line.length();
    double normalisedDist = (distance/float(this->posLog[index].size()-step));

    qDebug() << normalisedDist;

    if (normalisedDist < robotRadius*0.075){
        qDebug() << index << "TOO SLOW!!";
        speed = (normalisedDist < robotRadius*0.025)? +3 : +1;
        return false;
    }
    else if (normalisedDist > robotRadius*0.15){
        qDebug() << index << "TOO QUICK!!";
        speed = -1;
        return false;
    }
    else {
        qDebug() << index << "GOOD SPEED!!";
        return true;
    }
}

double KilobotCalibrateEnv::euclideanDist(const cv::Point& a, const cv::Point& b) {
    cv::Point diff = a - b;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

// Rename:
void KilobotCalibrateEnv::updateVirtualSensor(Kilobot kilobot)
{


    QLineF kilobot_vect = QLineF(kilobot.getPosition(), kilobot.getPosition() + kilobot.getVelocity());

    if (kilobot.getID() > this->isInit.size() - 1) {
        this->isInit.resize(kilobot.getID()+1);
        this->left_right.resize(kilobot.getID()+1);
        this->posLog.resize(kilobot.getID()+1);
        this->posLogTimes.resize(kilobot.getID()+1);
        this->velocityLog.resize(kilobot.getID()+1);
        this->commandLog.resize(kilobot.getID()+1);
        this->times.resize(kilobot.getID()+1);
        this->calibrationStage.resize(kilobot.getID()+1);
        this->noGoodSpeedCounter.resize(kilobot.getID()+1);
    }
    // init
    if (this->isInit[kilobot.getID()] == false) {
        this->isInit[kilobot.getID()] = true;
        this->left_right[kilobot.getID()] = QPoint(60,60);
        this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
        this->commandLog[kilobot.getID()] = LEFT;
        this->calibrationStage[kilobot.getID()] = DETECTING_MOVE;
        times[kilobot.getID()].start();
        num_kilobots++;

        kilobot_message msg;
        msg.id = kilobot.getID();
        msg.type = LEFT;
        msg.data = this->left_right[kilobot.getID()].x();
        emit transmitKiloState(msg);
    }

    this->posLog[kilobot.getID()].push_back(kilobot.getPosition());
    this->posLogTimes[kilobot.getID()].push_back(times[kilobot.getID()].elapsed());

    /* Time passed from last command sent */
    int timeInterval = times[kilobot.getID()].elapsed();
    /* Timeframes to wait before evaluating possible move occurred */
    int minTimeForEstimatingMove = 1000;
    /* Timeframes to wait before evaluating the speed of the robot in doing a revolution */
    int minTimeForEstimatingRevSpeed = 8;
    //int framesForMinEstRevSpeed = minTimeForEstimatingRevSpeed / 250;
    /* Max timeframes waited without noticing enough movement or limit for incorrect speed */
    int maxTimeToDetectMove = 10;
    /* Max timeframes waited without completing a revolution */
    int maxTimeToDetectRevolution = 15000;//ceil((revolutionTimeSecondsUpperBound*2)/frameLengthSeconds);
    bool revolutionCompleted = false;
    bool goodTrajectory = false;
    double revolutionTimeSeconds = 0;

    std::vector < cv::Point >  lastTrajData;

    for (int i = 0; i < this->posLog[kilobot.getID()].size(); ++i)
    {
        lastTrajData.push_back(cv::Point(posLog[kilobot.getID()][i].x(), posLog[kilobot.getID()][i].y()));
    }

    double robotRadius = 8.0f;

    int speed = 0;

    if (this->calibrationStage[kilobot.getID()] == DONE) {
        if (this->commandLog[kilobot.getID()] == LEFT) {
            this->commandLog[kilobot.getID()] = RIGHT;
            this->calibrationStage[kilobot.getID()] = DETECTING_MOVE;
        }  else if (this->commandLog[kilobot.getID()] == RIGHT) {
            this->commandLog[kilobot.getID()] = DONE_MOTION;
            num_done++;
            if (num_done == num_kilobots) {
                this->rotDone = true;
                // now do straight
            }
        }
        else {
            return;
        }
    }

    qDebug() << kilobot.getID() << this->left_right[kilobot.getID()].x();


    switch (this->calibrationStage[kilobot.getID()]){
    /* Here, we check if the robot makes any movement at a decent speed which SHOULD be computed as a function of timeframe and desired speed */
    case DETECTING_MOVE:{
        if (timeInterval > minTimeForEstimatingMove){
            bool goodSpeed = evaluateSpeed(0, timeInterval, robotRadius, speed, kilobot.getID());
            if (goodSpeed){
                calibrationStage[kilobot.getID()] = EVALUATING_REV_SPEED;
            }
        }
        if (timeInterval > maxTimeToDetectMove*250){
            /* Sending a new command because the robot is not moving correctly */
            this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
            kilobot_message msg;
            msg.id = kilobot.getID();
            msg.type = this->commandLog[kilobot.getID()];
            msg.data = this->left_right[kilobot.getID()].x();
            emit transmitKiloState(msg);
            times[kilobot.getID()].start();
            this->posLog[kilobot.getID()].clear();
            this->posLogTimes[kilobot.getID()].clear();
            this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
            //return true; // true sends a new message
        }
        break;
    }
    /* Here, we check if the trajectory of the robot is correct (a circle-like ellipse) and if the time of revolution is about 15s */
    case EVALUATING_REV_SPEED:{
        if (timeInterval > 2500){
            int lastVal = 0;
            for (uint i = this->posLog[kilobot.getID()].size() - 1; i > 0; --i) {
                if (timeInterval - this->posLogTimes[kilobot.getID()][i] > 2500) {
                    lastVal = i;
                    break;
                }
            }
            bool goodSpeed = evaluateSpeed(lastVal, timeInterval, robotRadius, speed, kilobot.getID());
            if (!goodSpeed) {
                noGoodSpeedCounter[kilobot.getID()]++;
                if (noGoodSpeedCounter[kilobot.getID()] >= maxTimeToDetectMove) {
                    if (speed != 0) {
                        this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
                        kilobot_message msg;
                        msg.id = kilobot.getID();
                        msg.type = this->commandLog[kilobot.getID()];
                        msg.data = this->left_right[kilobot.getID()].x();
                        emit transmitKiloState(msg);
                        times[kilobot.getID()].start();
                        this->posLog[kilobot.getID()].clear();
                        this->posLogTimes[kilobot.getID()].clear();
                        this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
                    }
                }
                break;
            }
            noGoodSpeedCounter[kilobot.getID()] = 0;
            cv::RotatedRect fittedEllipse = cv::fitEllipse(lastTrajData);
            /* We need that the ellipse size (diameters) are bigger than the robot radius and smaller than the robot 1.5*diameter */
            if (fittedEllipse.size.width > robotRadius && fittedEllipse.size.width < robotRadius*3 &&
                    fittedEllipse.size.height > robotRadius && fittedEllipse.size.height < robotRadius*3){
                /* the ellipse must be very similar to a circle (ratio between 0.9 and 1.1) */
                double sizeRatio = fittedEllipse.size.width/fittedEllipse.size.height;
                if (sizeRatio > 0.9 && sizeRatio <= 1.1) {
                    qDebug() << kilobot.getID() << "Good trajectory!";
                    goodTrajectory = true;
                }
            }

            /* Here, checking if the robot has completed a revolution */
            QLineF kilobot_old_vect = QLineF(QPointF(0,0), this->velocityLog[kilobot.getID()]);
            qreal ang = kilobot_old_vect.angleTo(kilobot_vect);
            if (ang > 180.0f) ang -= 360.0f;
            qDebug() << kilobot.getID() << qAbs(ang);
            if (qAbs(ang) < 20.0f) {
                revolutionCompleted = true;
            }
            revolutionTimeSeconds = timeInterval;
        }
        break;
    }
    case DONE:
    {
        break;
    }
    }

    if (revolutionCompleted){
        qDebug() << kilobot.getID() << "* * * REVOLUTION DONE IN " << timeInterval << " frames that are " << revolutionTimeSeconds << "s" ;
        revolutionCompleted = true;
        if (revolutionTimeSeconds > 25000) {
            qDebug() << kilobot.getID() << "TOO SLOW!" ;
            speed = +1;
        }
        else if (revolutionTimeSeconds < 10000) {
            qDebug() << kilobot.getID() << "TOO QUICK!" ;
            speed = -1;
        } else {
            qDebug() << "* * * * * * * * * * * * * * * * * * *" ;
            qDebug() << "* * * * * CALIBRATION DONE! * * * * *" << kilobot.getID() ;
            qDebug() << "* * * * * * * * * * * * * * * * * * *" ;
            speed = 0;
            this->calibrationStage[kilobot.getID()] = DONE;
            kilobot_message msg;
            msg.id = kilobot.getID();
            msg.type = STOP;
            msg.data = 0;
            emit transmitKiloState(msg);
        }
        //qDebug() << "Sending a new command or terminate calibration." ;
        if (speed != 0) {
            this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
            kilobot_message msg;
            msg.id = kilobot.getID();
            msg.type = this->commandLog[kilobot.getID()];
            msg.data = this->left_right[kilobot.getID()].x();
            emit transmitKiloState(msg);
            times[kilobot.getID()].start();
            this->posLog[kilobot.getID()].clear();
            this->posLogTimes[kilobot.getID()].clear();
            this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
        }
    }
    /* if for too long time the revolution is not completed, either:
     * the robot is not doing a circle (i.e., !goodTrajectory) thus we decrease speed
     * the robot does a circle (i.e., goodTrajectory) but moves too slow (?) */
    if (timeInterval > maxTimeToDetectRevolution){
        bool goodSpeed = evaluateSpeed(0, timeInterval, robotRadius, speed, kilobot.getID());
        if (goodSpeed && goodTrajectory) {
            qDebug() << "Trajectory and avg speed was good but the robot is completing the revolution slowly (speed up +1)" ;
            speed = +1;
        } else {
            if (goodSpeed){
                //std::cout << "Speed was good, but Trajectory was not circular, thus probably it was too quick [NOT SURE]." ;
                qDebug() << "Speed was good, but Trajectory was not circular, thus I randomly add +2 to test new values." ;
                speed = +2;
            }
        }
        if (speed != 0) {
            this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
            kilobot_message msg;
            msg.id = kilobot.getID();
            msg.type = this->commandLog[kilobot.getID()];
            msg.data = this->left_right[kilobot.getID()].x();
            emit transmitKiloState(msg);
            times[kilobot.getID()].start();
            this->posLog[kilobot.getID()].clear();
            this->posLogTimes[kilobot.getID()].clear();
            this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
        }
    }
}



/*void KilobotCalibrateEnv::updateVirtualSensor(Kilobot kilobot)
{


    //
    QLineF kilobot_vect = QLineF(kilobot.getPosition(), kilobot.getPosition() + kilobot.getVelocity());

    //bool first = false;

    if (kilobot.getID() > this->left_right.size() - 1) {
        this->left_right.resize(kilobot.getID()+1);
        this->dual_offset.resize(kilobot.getID()+1);
        this->velocityLog.resize(kilobot.getID()+1);
        this->commandLog.resize(kilobot.getID()+1);
        this->count.resize(kilobot.getID()+1);
        this->times.resize(kilobot.getID()+1);
    }
    // init
    if (this->left_right[kilobot.getID()].x() == 0) {
        this->left_right[kilobot.getID()] = QPoint(40,40);
        this->dual_offset[kilobot.getID()] = 20;
        this->commandLog[kilobot.getID()] = STOP;
        count[kilobot.getID()] = 0;
        times[kilobot.getID()].start();
        //first = true;
    }

    if (stage == 0) {

        // ROUGH CALIBRATE LEFT
        if (count[kilobot.getID()] == 0) {

            if (this->commandLog[kilobot.getID()] == STOP) {
                times[kilobot.getID()].start();
                kilobot_message msg;
                msg.id = kilobot.getID();
                msg.type = LEFT;
                msg.data = this->left_right[kilobot.getID()].x();
                emit transmitKiloState(msg);
                this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
                this->commandLog[kilobot.getID()] = LEFT;
            }

            if (times[kilobot.getID()].elapsed() > 1000 && times[kilobot.getID()].elapsed() < 1300) { // check
                qDebug() << kilobot.getID() << this->left_right[kilobot.getID()] << times[kilobot.getID()].elapsed();
                // test rotation
                QLineF old_vect(QPointF(0,0), this->velocityLog[kilobot.getID()]);
                qreal ang2 = old_vect.angleTo(kilobot_vect);
                qDebug() << kilobot.getID() << ang2;
                if (ang2 > 180.0f) ang2 -= 360.0f;
                if (ang2 < 10.0f || old_vect.length() < 0.01) { // quick check for any movement
                    this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+1);
                    kilobot_message msg;
                    msg.id = kilobot.getID();
                    msg.type = STOP;
                    msg.data = 0;
                    emit transmitKiloState(msg);
                    this->commandLog[kilobot.getID()] = STOP;
                }
            }
            if (times[kilobot.getID()].elapsed() > 7500) {
                qDebug() << kilobot.getID() << this->left_right[kilobot.getID()] << times[kilobot.getID()].elapsed();
                //times[kilobot.getID()].start();
                // test rotation
                QLineF old_vect(QPointF(0,0), this->velocityLog[kilobot.getID()]);
                qreal ang2 = old_vect.angleTo(kilobot_vect);
                if (ang2 > 180.0f) ang2 -= 360.0f;
                if (ang2 > -90.0f || old_vect.length() < 0.01) { // if > -90 then hasn't gone far enough to flip from +ve to -ve (i.e. less than 180deg)
                    this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+1);
                    kilobot_message msg;
                    msg.id = kilobot.getID();
                    msg.type = STOP;
                    msg.data = 0;
                    emit transmitKiloState(msg);
                    this->commandLog[kilobot.getID()] = STOP;
                } else {
                    // yay - left is done
                    kilobot_message msg;
                    msg.id = kilobot.getID();
                    msg.type = STOP;
                    msg.data = 0;
                    emit transmitKiloState(msg);
                    ++num_done;
                    if (num_done == count.size()) {
                        stage = 1;
                        num_done = 0;
                        count[kilobot.getID()] = 1;
                        qDebug() << "LEFT DONE" << this->left_right[kilobot.getID()].x();
                    } else {
                        count[kilobot.getID()] = 1;
                        qDebug() << kilobot.getID() << "LEFT DONE" << this->left_right[kilobot.getID()].x();
                    }
                }
                // store for next time
                this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
            }
        }


    } else if (stage == 1) {

        // ROUGH CALIBRATE RIGHT
        if (count[kilobot.getID()] == 1) {

            if (this->commandLog[kilobot.getID()] == STOP) {
                times[kilobot.getID()].start();
                kilobot_message msg;
                msg.id = kilobot.getID();
                msg.type = RIGHT;
                msg.data = this->left_right[kilobot.getID()].y();
                emit transmitKiloState(msg);
                this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
                this->commandLog[kilobot.getID()] = RIGHT;
            }

            if (times[kilobot.getID()].elapsed() > 1000 && times[kilobot.getID()].elapsed() < 1300) { // check
                qDebug() << kilobot.getID() << this->left_right[kilobot.getID()] << times[kilobot.getID()].elapsed();
                // test rotation
                QLineF old_vect(QPointF(0,0), this->velocityLog[kilobot.getID()]);
                qreal ang2 = old_vect.angleTo(kilobot_vect);
                if (ang2 > 180.0f) ang2 -= 360.0f;
                if (ang2 > -10.0f || old_vect.length() < 0.01) { // quick check for any movement
                    this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+1);
                    kilobot_message msg;
                    msg.id = kilobot.getID();
                    msg.type = STOP;
                    msg.data = 0;
                    emit transmitKiloState(msg);
                    this->commandLog[kilobot.getID()] = STOP;
                }
            }
            if (times[kilobot.getID()].elapsed() > 7500) {
                // test rotation
                qDebug() << kilobot.getID() << this->left_right[kilobot.getID()] << times[kilobot.getID()].elapsed();
                times[kilobot.getID()].start();
                QLineF old_vect(QPointF(0,0), this->velocityLog[kilobot.getID()]);
                qreal ang2 = old_vect.angleTo(kilobot_vect);
                if (ang2 > 180.0f) ang2 -= 360.0f;
                if (ang2 < 90.0f) {
                    this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+1);
                    kilobot_message msg;
                    msg.id = kilobot.getID();
                    msg.type = STOP;
                    msg.data = 0;
                    emit transmitKiloState(msg);
                    this->commandLog[kilobot.getID()] = STOP;
                } else {
                    // yay - left is done
                    kilobot_message msg;
                    msg.id = kilobot.getID();
                    msg.type = STOP;
                    msg.data = 0;
                    emit transmitKiloState(msg);
                    ++num_done;
                    if (num_done == count.size()) {
                        stage = 2;
                        num_done = 0;
                        this->rotDone = true;
                        count[kilobot.getID()] = 2;
                        for (int i = 0; i < this->times.size(); ++i) {
                            this->times[i].start();
                        }
                        qDebug() << "RIGHT DONE" << this->left_right[kilobot.getID()].y();
                    } else {
                        count[kilobot.getID()] = 2;
                        qDebug() << kilobot.getID() << "RIGHT DONE" << this->left_right[kilobot.getID()].y();
                    }
                }
                // store for next time
                this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
            }
        }

    } else if (stage == 2) {

       // CALIBRATE STRAIGHT

       // initial straight command
       if (count[kilobot.getID()] == 2 && times[kilobot.getID()].elapsed() > 1000) {
           this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()-5);
           this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()-5);
           kilobot_message msg;
           msg.id = kilobot.getID();
           msg.type = STRAIGHT_L;
           msg.data = this->left_right[kilobot.getID()].x();
           emit transmitKiloState(msg);
           kilobot_message msg2;
           msg2.id = kilobot.getID();
           msg2.type = STRAIGHT_R;
           msg2.data = this->left_right[kilobot.getID()].y();
           emit transmitKiloState(msg2);
           this->commandLog[kilobot.getID()] = STRAIGHT_L;
           count[kilobot.getID()] = 3;
           times[kilobot.getID()].start();
           this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
       }
       else if (count[kilobot.getID()] == 3) {
           if (times[kilobot.getID()].elapsed() > 3000) {
               times[kilobot.getID()].start();
               // get angle between prev and curr directions
               QLineF old_vect(QPointF(0,0), this->velocityLog[kilobot.getID()]);
               qreal ang2 = old_vect.angleTo(kilobot_vect);
               if (kilobot_vect.length() > 1.0f) {
                   if (ang2 > 180.0f) ang2 -= 360.0f;
                   if (ang2 > 20.0f) {

                        qDebug() << kilobot.getID() << "IS LEANING LEFT";
                        this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+1);
                        kilobot_message msg;
                        msg.id = kilobot.getID();
                        msg.type = STRAIGHT_R;
                        msg.data = this->left_right[kilobot.getID()].y();
                        emit transmitKiloState(msg);

                   } else if (ang2 < -20.0f) {

                       qDebug() << kilobot.getID() << "IS LEANING RIGHT";
                       this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+1);
                       kilobot_message msg;
                       msg.id = kilobot.getID();
                       msg.type = STRAIGHT_L;
                       msg.data = this->left_right[kilobot.getID()].x();
                       emit transmitKiloState(msg);

                   } else {
                       ++ num_done;
                       count[kilobot.getID()] = 4;
                       qDebug() << kilobot.getID() << "IS STRAIGHT";
                       kilobot_message msg;
                       msg.id = kilobot.getID();
                       msg.type = STOP;
                       msg.data = 0;
                       emit transmitKiloState(msg);
                       if (num_done == count.size()) {
                           this->strDone = true;
                       }
                   }
               } else {
                   qDebug() << kilobot.getID() << "IS SLOW";
                   this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+1);
                   kilobot_message msg;
                   msg.id = kilobot.getID();
                   msg.type = STRAIGHT_L;
                   msg.data = this->left_right[kilobot.getID()].x();
                   emit transmitKiloState(msg);
                   this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+1);
                   kilobot_message msg2;
                   msg2.id = kilobot.getID();
                   msg2.type = STRAIGHT_R;
                   msg2.data = this->left_right[kilobot.getID()].y();
                   emit transmitKiloState(msg2);
               }
               this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
           }
       }


    }



}*/




