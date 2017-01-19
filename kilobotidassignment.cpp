#include "kilobotidassignment.h"
#include <QDebug>

KilobotIDAssignment::KilobotIDAssignment()
{

}


void KilobotIDAssignment::initialise(bool)
{

    emit clearDrawings();

    this->serviceInterval = 200;

    // calibrate LEDs
    emit setTrackingType(ADAPTIVE_LED);
    emit getInitialKilobotStates();
    kilobot_broadcast msg;
    msg.type = 10;
    emit broadcastMessage(msg);
    this->time = 0.0;
    this->lastTime = 0.0;
    this->numFound = 0;
    this->numSegments = 0;
    this->switchSegment = true;
    this->stage = START;
    qDebug() << "INIT";
    t.start();
}

void KilobotIDAssignment::run()
{
    this->time += 0.1; // 100 ms in sec
    this->lastTime += 0.1;

    // stop broadcasting (replace)
    kilobot_broadcast msg;
    msg.type = 20;
    emit broadcastMessage(msg);

    if (time > 2.0f && time < 2.15f) {
        // finish adaptation
        emit setTrackingType(LED);
        qDebug() << "CHANGE TRACKING TYPE";
    }

    if (time > 2.0f) {

        switch (this->stage) {
        case START:
        {
            qDebug() << "START" << t.elapsed();
            this->lastTime = 0;
            this->switchSegment = true;

            this->stage = TEST;
            this->numFound = 0;
            this->numSegments = 0;
            for (int i = 0; i < tempIDs.size(); ++i) {

                // reset dupes
                if (this->tempIDs[i] == DUPE) this->tempIDs[i] = 0;

            }
            // get num
            kilobot_broadcast msg;
            msg.type = 1;
            emit broadcastMessage(msg);

            break;
        }
        case TEST:
        {

            if (this->switchSegment == true) {

                qDebug() << "SWITCH" << lastTime << t.elapsed();

                this->switchSegment = false;

                // when we have all segments
                if (numSegments > 10) {
                    for (int i = 0; i < tempIDs.size(); ++i) {
                        qDebug() << i << ":" << this->tempIDs[i];
                    }
                    this->dupesFound = false;
                    // work out if we have have duplicates, if so fix them
                    for (int i = 0; i < this->tempIDs.size(); ++i) {
                        // if we haven't already been identified as a dupe, or assigned
                        if (tempIDs[i] != DUPE && !this->isAssigned[i]) {
                            // already compared to ones below
                            int temp = tempIDs[i];
                            for (int j = i+1; j < this->tempIDs.size(); ++j) {
                                if (temp == tempIDs[j]) {
                                    tempIDs[i] = DUPE;
                                    tempIDs[j] = DUPE;
                                    // signal dupes
                                    this->dupesFound = true;
                                }
                            }
                        }
                    }

                    this->stage = SEND;

                }

                // get next seg
                kilobot_broadcast msg;
                msg.type = 4;
                emit broadcastMessage(msg);
            }
            if (lastTime > 1.0f*float(numSegments+1)+0.21f) {

                ++numSegments;
                emit updateKilobotStates();
                this->switchSegment = true;

            }

            break;
        }
        case SEND:
        {
            if (lastTime > 16.0f) {
                qDebug() << "SEND" << lastTime;
                if (this->tempIDs[numFound] != DUPE && !this->isAssigned[numFound]) {
                    QVector<uint8_t> data;
                    data.resize(9);
                    // current temp ID
                    data[0] = (this->tempIDs[numFound] >> 8)&0xFF;
                    data[1] = this->tempIDs[numFound]&0xFF;
                    // UID to set
                    data[2] = (numFound >> 8)&0xFF;
                    data[3] = numFound&0xFF;
                    kilobot_broadcast msg;
                    msg.type = 2;
                    msg.data = &data[0];
                    emit broadcastMessage(msg);
                    this->isAssigned[numFound] = true; // set as assigned
                }
                ++numFound;
                if (numFound > tempIDs.size() - 1) {
                    if (dupesFound) {
                        this->stage = RETRY;
                    } else {
                        // end
                        this->stage = COMPLETE;
                    }
                }
            }
            break;
        }
        case RETRY:
        {
            qDebug() << "RETRY" << lastTime;
            kilobot_broadcast msg;
            msg.type = 3;
            emit broadcastMessage(msg);
            this->stage = START;
            break;
        }
        case COMPLETE:
        {

            qDebug() << "DONE" << lastTime;
            emit experimentComplete();
            break;
        }
        }

    }

}

// run once for each kilobot after emitting getInitialKilobotStates() signal
void KilobotIDAssignment::setupInitialKilobotState(Kilobot kilobotCopy)
{

    // resize and insert kilobot ID
    if (kilobotCopy.getID()+1 > this->tempIDs.size()) {
        this->tempIDs.resize(kilobotCopy.getID() + 1);
        this->isAssigned.resize(kilobotCopy.getID() + 1);
    }
    this->tempIDs[kilobotCopy.getID()] = DUPE;
    this->isAssigned[kilobotCopy.getID()] = false;

}

// run once for each kilobot after emitting updateKilobotStates() signal
void KilobotIDAssignment::updateKilobotState(Kilobot kilobotCopy)
{

    //qDebug() << "here2";
    // check colours

    if (isAssigned[kilobotCopy.getID()]) return;

    lightColour col = kilobotCopy.getLedColour();

    int col2 = 0;

    if (col == RED) {
        col2 = 0;
    }
    if (col == BLUE) {
        col2 = 1;
    } else {
        qDebug() << "DETECTION ERROR";
    }

    this->tempIDs[kilobotCopy.getID()] += int(col) * binaryMultipliers[numSegments-1];
    QString colName;
    if (col == OFF) colName = "OFF";
    if (col == RED) colName = "RED";
    if (col == GREEN) colName = "GREEN";
    if (col == BLUE) colName = "BLUE";
    qDebug() << "KB" << kilobotCopy.getID() << "part " << numSegments -1 << " = " << colName << t.elapsed();


}
