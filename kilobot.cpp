/*
 * Kilobot.cpp
 *
 *  Created on: 16 Sep 2016
 *      Author: marshall
 */


#include "kilobotenvironment.h"
#include "kilobot.h"
#include <assert.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <QDebug>
#include <QLineF>


ColourBuffer::ColourBuffer(int size){
    buffer_size = size;
    buffer.resize(size);
}

void ColourBuffer::addColour(lightColour newColour){
    buffer.removeFirst();
    buffer.push_back(newColour);
    //qDebug() << "ADDED NEW COLOUR!! " << newColour;
}

lightColour ColourBuffer::getAvgColour(){
//    int countOFF = 0;
//    int countRED = 0;
//    int countGREEN = 0;
//    int countBLUE = 0;
    std::vector <int> counters;
    counters.resize(4); // OFF, RED, GREEN, BLUE
    for (int i = 0; i < buffer.size(); ++i){
        switch (buffer.at(i)) {
        case (OFF):{
            counters[0]++;
//            countOFF++;
            break;
        }
        case (RED):{
            counters[1]++;
//            countRED++;
            break;
        }
        case (GREEN):{
            counters[2]++;
//            countGREEN++;
            break;
        }
        case (BLUE):{
            counters[3]++;
//            countBLUE++;
            break;
        }
        }
    }
    double * minVal = new double;
    double * maxVal = new double;
    int * maxLoc = new int;
    cv::minMaxIdx(counters, minVal, maxVal, NULL, maxLoc);
//    qDebug() << "buffer" << buffer << "counters" << counters;
//    qDebug() << "maxLoc" << *maxLoc << "[0]" << maxLoc[0] << "[1]" << maxLoc[1];
    switch (maxLoc[1]) {
    case (0):{
//        qDebug() << " returning OFF";
        return OFF;
        break;
    }
    case (1):{
//        qDebug() << " returning RED";
        return RED;
        break;
    }
    case (2):{
//        qDebug() << " returning GREEN";
        return GREEN;
        break;
    }
    case (3):{
//        qDebug() << " returning BLUE";
        return BLUE;
        break;
    }
    }
}

void OrientationBuffer::addOrientation(QPointF newOrientation){
    while (buffer.size() >= buffer_size){
        buffer.removeFirst();
    }
    buffer.push_back(newOrientation);
    //qDebug() << "ADDED NEW ORIENTATION " << newOrientation;
}

QPointF OrientationBuffer::getAvgOrientation(){
    float totWeights = 0;
    QPointF weightedOrientation(0,0);
    for (int i = 0; i < buffer.size(); ++i){
        float weight = ((float)(i+1))/(float)buffer.size();
        totWeights += weight;
        weightedOrientation += (weight * buffer[i]);
        //        qDebug() << "item is " << buffer[i] << "weighted becomes:" << weight * buffer[i] << "sum to now is" << weightedOrientation;
    }
    //    qDebug() << "FINAL(" << totWeights << ")" << weightedOrientation/totWeights;
    return weightedOrientation/totWeights;
}

void PositionBuffer::addPosition(QPointF newPosition){
    while (buffer.size() >= buffer_size){
        buffer.removeFirst();
    }
    buffer.push_back(newPosition);
    //qDebug() << "ADDED NEW POS " << newPosition;
}

QPointF PositionBuffer::getOrientationFromPositions() {
    QPointF orientation;
    if (buffer.size() > 1){
        QPointF first = buffer.first();
        QPointF last = buffer.last();
        if (buffer.size() > 2){
            first += buffer.at(1);
            first /= 2.0;
            last += buffer.at(buffer.size()-2);
            last /= 2.0;
        }
        orientation = (last - first)/buffer.size();
    }
    return orientation;
}

Kilobot::Kilobot(kilobot_id identifier, QPointF position, QPointF velocity, kilobot_colour colourValues) {
    // TODO Auto-generated constructor stub
    //assert(id <= pow(2, uint8_t_LENGTH) - 1); // we don't need these now we have a bitfield structure ;-)
    id = identifier;
    pos = position;
    vel = velocity;
    col = colourValues;

}

Kilobot::~Kilobot() {
    // TODO Auto-generated destructor stub
}

// copy assignment
Kilobot::Kilobot(const Kilobot& other)
{
    if (this != &other) {
        this->vel = other.vel;
        this->pos = other.pos;
        this->id = other.id;
        this->col = other.col;
    }
}

void Kilobot::updateHardware()
{
    Kilobot copyOfMe(*this);
    emit sendUpdateToHardware(copyOfMe);
}

void Kilobot::updateExperiment()
{
    Kilobot copyOfMe = (*this);
    emit sendUpdateToExperiment(this, copyOfMe);
}

void Kilobot::updateState(QPointF position, QPointF velocity, kilobot_colour colourValues) {
    /*assert(colourValues.r <= KILOBOT_MAX_COLOUR);
    assert(colourValues.g <= KILOBOT_MAX_COLOUR);
    assert(colourValues.b <= KILOBOT_MAX_COLOUR);*/

    vel = velocity;
    pos = position;
    col = colourValues;

}

QPointF Kilobot::getPosition()
{
    return this->pos;
}

QPointF Kilobot::getVelocity()
{
    return this->vel;
}

kilobot_colour Kilobot::getLedColour()
{
    return this->col;
}

kilobot_id Kilobot::getID()
{
    return this->id;
}

void Kilobot::setID(kilobot_id id)
{
    this->id = id;
}
