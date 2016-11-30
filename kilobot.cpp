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

Kilobot::Kilobot(uint8_t identifier, QPointF position, QPointF velocity, kilobot_colour colourValues) {
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

uint8_t Kilobot::getID()
{
    return this->id;
}

void Kilobot::setID(uint8_t id)
{
    this->id = id;
}
