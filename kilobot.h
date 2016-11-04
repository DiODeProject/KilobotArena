/*
 * Kilobot.h
 *
 *  Created on: 16 Sep 2016
 *      Author: marshall
 */

#ifndef KILOBOT_H_
#define KILOBOT_H_

typedef unsigned int kilobot_id;
typedef unsigned int kilobot_pos;
typedef unsigned char kilobot_channel_colour;
typedef unsigned char kilobot_message_type;
typedef unsigned int kilobot_message_data;

#define KILOBOT_ID_LENGTH 10 // in bits
#define KILOBOT_MESSAGE_TYPE_LENGTH 4 // in bits
#define KILOBOT_MESSAGE_DATA_LENGTH 10 // in bits
#define KILOBOT_MAX_COLOUR 255 // colour channel
#define KILOBOT_REFRESH_DELAY 3 // in milliseconds

#define KILOBOT_MAX_ID 1024 //

// for now
typedef unsigned int KilobotEnvironment;


struct kilobot_colour
{
    kilobot_channel_colour r;
    kilobot_channel_colour g;
    kilobot_channel_colour b;
};

class Kilobot {
public:
    Kilobot(kilobot_id identifier, kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment environment);
    Kilobot() {}
    ~Kilobot();
    kilobot_id getID();
    void setID(kilobot_id);
    kilobot_pos getXPosition();
    kilobot_pos getYPosition();
    kilobot_colour getLedColour();
    //kilobot_colour resolveKilobotState(stateColours);
    void updateState(kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment environment);
    kilobot_message_data getEnvironmentValue();

    // temporary
    int lightThreshold = 220;

private:
    kilobot_id id = 0;
    kilobot_pos x = 0;
    kilobot_pos y = 0;
    kilobot_colour col = {0,0,0};
    KilobotEnvironment env = 0;
};

#endif // KILOBOT_H
