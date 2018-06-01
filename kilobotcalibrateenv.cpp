#include "kilobotcalibrateenv.h"
#include <QVector>
#include <QLineF>
#include <QDebug>



#include "kilobot.h"

KilobotCalibrateEnv::KilobotCalibrateEnv(QObject *parent) : KilobotEnvironment(parent)
{
    qDebug() << "Calibrate selected";
    qDebug() << "Giovanni's heuristic";
}

// Only update if environment is dynamic:
void KilobotCalibrateEnv::update()
{

}


bool KilobotCalibrateEnv::evaluateSpeed(int step, int timeInterval, double robotRadius, int &speed, int index, command motionType){
    double distance = 0;
    for (int i = step; i < this->posLog[index].size()-1; ++i){
        distance += QLineF(this->posLog[index][i], this->posLog[index][i+1]).length();
    }
//    double normalisedDist = (distance/float(this->posLog[index].size()-step)); // computed using the number of entries (rather than timeInterval)
    double distOverSec = distance*1000.0/timeInterval;

    double lowTresh, veryLowTresh;
    double highTresh, veryHighTresh;
    if (motionType == LEFT || motionType == RIGHT){
        lowTresh = robotRadius*0.70;
        veryLowTresh = robotRadius*0.25;
        highTresh = robotRadius*0.95;
        veryHighTresh = robotRadius*1.5;
    } else {
        lowTresh = robotRadius*0.70;
        veryLowTresh = robotRadius*0.25;
        highTresh = robotRadius*1.1;
        veryHighTresh = robotRadius*1.5;
    }


    qDebug() << "computed distOverSec:" << distOverSec << " rr:" << robotRadius << " low-th:" << lowTresh << " high-th:" << highTresh;

    if (distOverSec < lowTresh){
        qDebug() << index << "TOO SLOW(" << ((motionType == LEFT)?this->left_right[index].x():this->left_right[index].y()) << ")!!";
        speed = (distOverSec < veryLowTresh)? +3 : +1;
        return false;
    }
    else if (distOverSec > highTresh){
        qDebug() << index << "TOO QUICK(" << ((motionType == LEFT)?this->left_right[index].x():this->left_right[index].y()) << ")!!";
        speed = (distOverSec > veryHighTresh)? -5 : -1;
        return false;
    }
    else {
        qDebug() << index << "GOOD SPEED(" << ((motionType == LEFT)?this->left_right[index].x():this->left_right[index].y()) << ")!!";
        return true;
    }
}

double KilobotCalibrateEnv::euclideanDist(const cv::Point& a, const cv::Point& b) {
    cv::Point diff = a - b;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

//void KilobotCalibrateEnv::rotateVect(QPointF& vector2d, double angleInDeg) {
//    double radians = qDegreesToRadians(angleInDeg);
//    double x = vector2d.x()*qCos(radians) - vector2d.y()*qSin(radians);
//    double y = vector2d.x()*qSin(radians) - vector2d.y()*qCos(radians);
//    vector2d.setX(x);
//    vector2d.setY(y);
//}

void KilobotCalibrateEnv::updateVirtualSensor(Kilobot kilobot)
{
    //QLineF kilobot_vect = QLineF(kilobot.getPosition(), kilobot.getPosition() + kilobot.getVelocity());

    if (kilobot.getID() > this->isInit.size() - 1) {
        this->isInit.resize(kilobot.getID()+1);
        this->isInit[kilobot.getID()] = false;
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

        sendCalibMessage(kilobot.getID());
    }

    if (this->posLog[kilobot.getID()].empty()) this->velocityLog[kilobot.getID()] = kilobot.getVelocity();
    this->posLog[kilobot.getID()].push_back(kilobot.getPosition());
    this->posLogTimes[kilobot.getID()].push_back(times[kilobot.getID()].elapsed());
    if (this->commandLog[kilobot.getID()] == STRAIGHT_L) {
        std::vector<cv::Point> tmpPos;
        tmpPos.resize(this->posLog[kilobot.getID()].size());
        for (int i=0; i < this->posLog[kilobot.getID()].size(); ++i){
//            tmpPos.push_back( cv::Point(this->posLog[kilobot.getID()][i].x(),this->posLog[kilobot.getID()][i].y()) );
            tmpPos[i] = cv::Point(this->posLog[kilobot.getID()][i].x(),this->posLog[kilobot.getID()][i].y());
        }
        drawLine(tmpPos, QColor(0, 255, 0, 255), 2, "", false);
        tmpPos.clear();
        tmpPos.push_back( cv::Point(this->posLog[kilobot.getID()].first().x(),this->posLog[kilobot.getID()].first().y()) );
        tmpPos.push_back( cv::Point(this->posLog[kilobot.getID()].last().x(), this->posLog[kilobot.getID()].last().y()) );
        drawLine(tmpPos,  QColor(Qt::red), 2, "", false);
    }

    /* Time passed from last command sent (in ms) */
    int timeInterval = times[kilobot.getID()].elapsed();
    /* Timeframes to wait before evaluating possible move occurred */
    int minTimeForEstimatingMove = 1000;
    /* Timeframes to wait before evaluating the speed of the robot in doing a revolution */
    int minTimeForEstimatingRevQuality = 4000;
    int minTimeForEstimatingStraightQuality = 10000;
    int timeToConfirmStraightQuality = 15000;
    //int framesForMinEstRevSpeed = minTimeForEstimatingRevSpeed / 250;
    /* Max timeframes waited without noticing enough movement or limit for incorrect speed */
    int maxTimeToDetectMove = 2500;
    /* Max consecutive times the motion speed is bad during the second phase of revolution shape estimation */
    int maxCounterBadMotion = 10;
    /* Max timeframes waited without completing a revolution */
    int maxTimeToDetectRevolution = 15000;//ceil((revolutionTimeSecondsUpperBound*2)/frameLengthSeconds);
    /* Lower bound of revolution time (in ms) */
    int lowerBoundRevolution = 8000;
    /* Upper bound of revolution time (in ms) */
    int upperBoundRevolution = 15000;
    bool revolutionCompleted = false;
//    bool straightCompleted = false;
    bool goodTrajectory = false;
    double revolutionTimeSeconds = 0;

    std::vector < cv::Point >  lastTrajData;

    for (int i = 0; i < this->posLog[kilobot.getID()].size(); ++i)
    {
        lastTrajData.push_back(cv::Point(posLog[kilobot.getID()][i].x(), posLog[kilobot.getID()][i].y()));
    }

    int speed = 0;

    if (this->calibrationStage[kilobot.getID()] == DONE) {
        if (this->commandLog[kilobot.getID()] == LEFT) {
            this->commandLog[kilobot.getID()] = RIGHT;
            this->calibrationStage[kilobot.getID()] = DETECTING_MOVE;
        } else if (this->commandLog[kilobot.getID()] == RIGHT) {
            this->commandLog[kilobot.getID()] = STRAIGHT_L;
            this->calibrationStage[kilobot.getID()] = DETECTING_MOVE;
        } else if (this->commandLog[kilobot.getID()] == STRAIGHT_L) {
            this->commandLog[kilobot.getID()] = DONE_MOTION;
            num_done++;
            kilobot_message msg;
            msg.id = kilobot.getID();
            msg.type = STOP;
            msg.data = 0;
            emit transmitKiloState(msg);
            qDebug() << "STOP message sent to robot n." << kilobot.getID();
            if (num_done == num_kilobots) {
                this->rotDone = true;
                // now do straight
            }
        }
        else {
            return;
        }
    }

    // qDebug() << kilobot.getID() << this->left_right[kilobot.getID()].x();


    switch (this->calibrationStage[kilobot.getID()]){
    /* Here, we check if the robot makes any movement at a decent speed which SHOULD be computed as a function of timeframe and desired speed */
    case DETECTING_MOVE:{
//        qDebug() << "DETECTING MOVE timeInt:" << timeInterval;
        if (timeInterval > minTimeForEstimatingMove){
            bool goodSpeed = evaluateSpeed(0, timeInterval, this->kilobotRadius, speed, kilobot.getID(), this->commandLog[kilobot.getID()]);
            if (goodSpeed){
                calibrationStage[kilobot.getID()] = (this->commandLog[kilobot.getID()] == LEFT || this->commandLog[kilobot.getID()] == RIGHT)?
                            EVALUATING_REV_SPEED : EVALUATING_STRAIGHT_MOTION ;
            }
        }
        if (timeInterval > maxTimeToDetectMove){
            /* Sending a new command because the robot is not moving correctly */
            if (this->commandLog[kilobot.getID()] == LEFT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
            }
            if (this->commandLog[kilobot.getID()] == RIGHT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+speed);
            }
            sendCalibMessage(kilobot.getID());
        }
        break;
    }
        /* Here, we check if the trajectory of the robot is correct (a circle-like ellipse) and if the time of revolution is about 15s */
    case EVALUATING_REV_SPEED:{
        if (timeInterval > minTimeForEstimatingRevQuality){
            int lastVal = 0;
            int lastTime = timeInterval;
            for (int i = this->posLog[kilobot.getID()].size() - 1; i > 0; --i) {
                lastTime = timeInterval - this->posLogTimes[kilobot.getID()][i];
                if (lastTime > minTimeForEstimatingRevQuality) {
                    lastVal = i;
                    break;
                }
            }
            bool goodSpeed = evaluateSpeed(lastVal, lastTime, this->kilobotRadius, speed, kilobot.getID(), this->commandLog[kilobot.getID()]);
            if (!goodSpeed) {
                noGoodSpeedCounter[kilobot.getID()]++;
                if (noGoodSpeedCounter[kilobot.getID()] >= maxCounterBadMotion) {
                    if (speed != 0) {
                        if (this->commandLog[kilobot.getID()] == LEFT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                            this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
                        }
                        if (this->commandLog[kilobot.getID()] == RIGHT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                            this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+speed);
                        }
                        sendCalibMessage(kilobot.getID());
                    }
                }
                break;
            }
            noGoodSpeedCounter[kilobot.getID()] = 0;
            cv::RotatedRect fittedEllipse = cv::fitEllipse(lastTrajData);
            /* We need that the ellipse size (diameters) are bigger than the robot radius and smaller than the robot 1.5*diameter */
            if (fittedEllipse.size.width > this->kilobotRadius && fittedEllipse.size.width < this->kilobotRadius*3 &&
                    fittedEllipse.size.height > this->kilobotRadius && fittedEllipse.size.height < this->kilobotRadius*3){
                /* the ellipse must be very similar to a circle (ratio between 0.9 and 1.1) */
                double sizeRatio = fittedEllipse.size.width/fittedEllipse.size.height;
                if (sizeRatio > 0.9 && sizeRatio <= 1.1) {
                    qDebug() << kilobot.getID() << "Good trajectory!";
                    goodTrajectory = true;
                }
            }

            /* Here, checking if the robot has completed a revolution */
            //            QLineF kilobot_old_vect = QLineF(QPointF(0,0), this->velocityLog[kilobot.getID()]);
            //            qreal ang = kilobot_old_vect.angleTo(kilobot_vect);
            //            if (ang > 180.0f) ang -= 360.0f;
            //            QLineF initVel(this->posLog[kilobot.getID()][0], this->velocityLog[kilobot.getID()] );
            //            qreal initRot = initVel.angle();
            //            QLineF currentVel(this->posLog[kilobot.getID()].back(), kilobot.getVelocity() );
            //            qreal currentRot = currentVel.angle();
            //            double ang = qMin(360 - fabs(initRot - currentRot), fabs(initRot - currentRot));
            //            qDebug() << kilobot.getID() << qAbs(ang);
            //            if (qAbs(ang) < 20.0f) {
            //                revolutionCompleted = true;
            //            }
            qreal dist = QLineF(this->posLog[kilobot.getID()][0], this->posLog[kilobot.getID()].back()).length();
            // qDebug() << kilobot.getID() << "rev dist:" << dist;
            if (dist < this->kilobotRadius/4.0){
                revolutionCompleted = true;
            }
            revolutionTimeSeconds = timeInterval;
        }
        break;
    }
    /* Here, we check if the trajectory is straight */
    case EVALUATING_STRAIGHT_MOTION:{
        if (timeInterval > minTimeForEstimatingStraightQuality){
            double pixelPerSec = QLineF( this->posLog[kilobot.getID()].first(), this->posLog[kilobot.getID()].last() ).length() / timeInterval * 1000;
            bool goodSpeed = true;
            if (pixelPerSec < 9){
                goodSpeed = false;
                if ( evaluateSpeed(0, timeInterval, this->kilobotRadius, speed, kilobot.getID(), this->commandLog[kilobot.getID()]) || speed < 0){
                    // moving badly due to too high values
                    speed -= 2;
                    calibrationStage[kilobot.getID()] = DETECTING_MOVE;
                } else {
                    speed = +1;
                }
            }
            if (pixelPerSec > 14){
                goodSpeed = false;
                speed = -1;
            }
//            bool goodSpeed = evaluateSpeed(0, timeInterval, this->kilobotRadius, speed, kilobot.getID(), this->commandLog[kilobot.getID()]);
            if (!goodSpeed) {
                noGoodSpeedCounter[kilobot.getID()]++;
                if (noGoodSpeedCounter[kilobot.getID()] >= maxCounterBadMotion) {
                    if (speed != 0) {
                        this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
                        this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+speed);
                        sendCalibMessage(kilobot.getID());
                    }
                }
                break;
            }
            noGoodSpeedCounter[kilobot.getID()] = 0;
            double step_x = (this->posLog[kilobot.getID()].last().x() - this->posLog[kilobot.getID()].first().x()) / this->posLog[kilobot.getID()].size();
            double step_y = (this->posLog[kilobot.getID()].last().y() - this->posLog[kilobot.getID()].first().y()) / this->posLog[kilobot.getID()].size();
//            double coef = (this->posLog[kilobot.getID()].last().y() - this->posLog[kilobot.getID()].first().y()) / (this->posLog[kilobot.getID()].last().x() - this->posLog[kilobot.getID()].first().x());
//            double step = QLineF( this->posLog[kilobot.getID()].first(), this->posLog[kilobot.getID()].last() ).length() / this->posLog[kilobot.getID()].size();
            double dist_sum = 0;
            for (int i = 1; i < this->posLog[kilobot.getID()].size(); ++i){
                QPointF idealPoint = this->posLog[kilobot.getID()].first() + QPointF(step_x*i,step_y*i);
                dist_sum += QLineF(this->posLog[kilobot.getID()][i], idealPoint).length();
            }
            dist_sum /=  (this->posLog[kilobot.getID()].size()-1);

            /* We need that the ellipse size (diameters) are bigger than the robot radius and smaller than the robot 1.5*diameter */
            qDebug() << kilobot.getID() << " line dist is " << dist_sum;
            if (dist_sum < this->kilobotRadius*0.9){
                    qDebug() << kilobot.getID() << "Good trajectory (speed px/s:"<< pixelPerSec << ")!";
                    goodTrajectory = true;
                    if (goodTrajectory && timeToConfirmStraightQuality <= timeInterval){
        //                straightCompleted = true;
                        qDebug() << "* * * * * * * * * * * * * * * * * * *" ;
                        qDebug() << "* * * * CALIBRATION KB" <<kilobot.getID()<< "DONE!* * * *";
                        qDebug() << "* * * * * * * * * * * * * * * * * * *" ;
                        this->calibrationStage[kilobot.getID()] = DONE;
                    }
            } else {
                /* estimating if it's drifting left or right */
                // computing the angle of the vector start-to-end
//                QLineF straightLine( this->posLog[kilobot.getID()].last(), this->posLog[kilobot.getID()].first() );
//                QLineF midLine( this->posLog[kilobot.getID()][this->posLog[kilobot.getID()].size()/2], this->posLog[kilobot.getID()].first() );
                double straightLineAngle = QLineF( this->posLog[kilobot.getID()].last(), this->posLog[kilobot.getID()].first() ).angle();
                double midLineAngle = QLineF( this->posLog[kilobot.getID()].last(), this->posLog[kilobot.getID()].first() ).angle();
                double diffAngle = straightLineAngle - midLineAngle;
                while (diffAngle > 180) diffAngle -= 360;
                while (diffAngle <= -180) diffAngle += 360;
                if (diffAngle < 0){ // drifting to the right
//                    // increasing left value of +1
//                    this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+1);
                    // decreasing right value of -1
                    this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()-1);
                } else { // drifting to the left
//                    // increasing right value of +1
//                    this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+1);
                    // decreasing left value of -1
                    this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()-1);
                }
                sendCalibMessage(kilobot.getID());
            }

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
        if (revolutionTimeSeconds > upperBoundRevolution) {
            qDebug() << kilobot.getID() << "TOO SLOW!" ;
            speed = +1;
        }
        else if (revolutionTimeSeconds < lowerBoundRevolution) {
            qDebug() << kilobot.getID() << "TOO QUICK!" ;
            speed = -1;
        } else {
            qDebug() << "* * * * * * * * * * * * * * * * * * *" ;
            qDebug() << "* * * * CALIBRATION KB" <<kilobot.getID()<< "DONE!* * * *";
            qDebug() << "* * * * * * * * * * * * * * * * * * *" ;
            speed = 0;
            this->calibrationStage[kilobot.getID()] = DONE;
        }
        //qDebug() << "Sending a new command or terminate calibration." ;
        if (speed != 0) {
            if (this->commandLog[kilobot.getID()] == LEFT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
            }
            if (this->commandLog[kilobot.getID()] == RIGHT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+speed);
            }
            sendCalibMessage(kilobot.getID());
        }
    }
    /* if for too long time the revolution is not completed, either:
     * the robot is not doing a circle (i.e., !goodTrajectory) thus we decrease speed
     * the robot does a circle (i.e., goodTrajectory) but moves too slow (?) */
    if (timeInterval > maxTimeToDetectRevolution && (this->calibrationStage[kilobot.getID()] == LEFT || this->calibrationStage[kilobot.getID()] == RIGHT) ){
        bool goodSpeed = evaluateSpeed(0, timeInterval, this->kilobotRadius, speed, kilobot.getID(), this->commandLog[kilobot.getID()]);
        if (goodSpeed && goodTrajectory) {
            qDebug() << "Trajectory and avg speed was good but the robot is completing the revolution slowly (speed up +1)" ;
            speed = +1;
        } else {
            if (goodSpeed){
                qDebug() << "Speed was good, but Trajectory was not circular, thus probably it was too quick [NOT SURE] (-2)." ;
//                qDebug() << "Speed was good, but Trajectory was not circular, thus I randomly add +2 to test new values." ;
                speed = -2;
            }
        }
        if (speed != 0) {
            if (this->commandLog[kilobot.getID()] == LEFT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                this->left_right[kilobot.getID()].setX(this->left_right[kilobot.getID()].x()+speed);
            }
            if (this->commandLog[kilobot.getID()] == RIGHT || this->commandLog[kilobot.getID()] == STRAIGHT_L){
                this->left_right[kilobot.getID()].setY(this->left_right[kilobot.getID()].y()+speed);
            }
            sendCalibMessage(kilobot.getID());
        }
    }
}

void KilobotCalibrateEnv::sendCalibMessage(kilobot_id kID){
    /* composing the message */
    kilobot_message msg;
    msg.id = kID;
    msg.type = this->commandLog[kID];
    if (this->commandLog[kID] == LEFT || this->commandLog[kID] == STRAIGHT_L){
        msg.data = (uint16_t)(this->left_right[kID].x());
    }
    if (this->commandLog[kID] == RIGHT) {
        msg.data = (uint16_t)(this->left_right[kID].y());
    }
    emit transmitKiloState(msg);
    /* if we are calibrating the straight motion we need to send both left and right values */
    if (this->commandLog[kID] == STRAIGHT_L){
        msg.type = STRAIGHT_R;
        msg.data = (uint16_t)(this->left_right[kID].y());
        emit transmitKiloState(msg);
        qDebug() << "New fwd motion values (" << this->left_right[kID].x() << "," << this->left_right[kID].y() << ") sent to robot n." << kID;
    } else {
        qDebug() << "New rot motion value (" << this->left_right[kID].x() << ") sent to robot n." << kID;
    }
    /* reset counters and variables */
    times[kID].start();
    this->posLog[kID].clear();
    this->posLogTimes[kID].clear();
}





