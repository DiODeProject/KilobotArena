#include "kilobotcalibrateenv.h"
#include <QVector>
#include <QLineF>
#include <QDebug>

#include "kilobot.h"

KilobotCalibrateEnv::KilobotCalibrateEnv(QObject *parent) : KilobotEnvironment(parent)
{

    qDebug() << "Calibrate selected";

}

// Only update if environment is dynamic:
void KilobotCalibrateEnv::update()
{


}

// Rename:
void KilobotCalibrateEnv::updateVirtualSensor(Kilobot kilobot)
{


    //
    QLineF kilobot_vect = QLineF(kilobot.getPosition(), kilobot.getPosition() + kilobot.getVelocity());

    bool first = false;

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
        first = true;
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
                        for (uint i = 0; i < this->times.size(); ++i) {
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



}




