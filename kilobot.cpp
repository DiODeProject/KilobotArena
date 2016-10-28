/*
 * Kilobot.cpp
 *
 *  Created on: 16 Sep 2016
 *      Author: marshall
 */

#include "kilobot.h"
#include <assert.h>
#include <math.h>

Kilobot::Kilobot(kilobot_id identifier, kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment environment) {
    // TODO Auto-generated constructor stub
    assert(id <= pow(2, KILOBOT_ID_LENGTH) - 1);
    id = identifier;
    x = xPosition;
    y = yPosition;
    col = colourValues;
    env = environment;
}

Kilobot::~Kilobot() {
    // TODO Auto-generated destructor stub
}

//kilobot_colour Kilobot::resolveKilobotState(stateColours) {
    // TODO return stateColour from stateColours that has smallest Euclidean distance to this.col
//}

void Kilobot::updateState(kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment environment) {
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
