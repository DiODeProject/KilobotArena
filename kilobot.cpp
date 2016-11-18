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

Kilobot::Kilobot(uint8_t identifier, kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment * environment) {
    // TODO Auto-generated constructor stub
    //assert(id <= pow(2, uint8_t_LENGTH) - 1); // we don't need these now we have a bitfield structure ;-)
    id = identifier;
    x = xPosition;
    y = yPosition;
    col = colourValues;
    env = environment;
}

Kilobot::~Kilobot() {
    // TODO Auto-generated destructor stub
}

// copy assignment
Kilobot::Kilobot(const Kilobot& other)
{
    if (this != &other) {
        this->x = other.x;
        this->y = other.y;
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

void Kilobot::updateState(kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment * environment) {
    assert(colourValues.r <= KILOBOT_MAX_COLOUR);
    assert(colourValues.g <= KILOBOT_MAX_COLOUR);
    assert(colourValues.b <= KILOBOT_MAX_COLOUR);

    x = xPosition;
    y = yPosition;
    col = colourValues;
    if (env != 0)
    {
        env = environment;
    }
}

kilobot_pos Kilobot::getXPosition()
{
    return this->x;
}

kilobot_pos Kilobot::getYPosition()
{
    return this->y;
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
