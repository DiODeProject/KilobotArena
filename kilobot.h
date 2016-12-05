/*
 * Kilobot.h
 *
 *  Created on: 16 Sep 2016
 *      Author: marshall
 */

#ifndef KILOBOT_H_
#define KILOBOT_H_

#include <stdint.h>

//typedef unsigned int uint8_t;
typedef unsigned int kilobot_pos;
typedef unsigned char kilobot_channel_colour;
//typedef unsigned char kilobot_message_type;
typedef struct {
        uint8_t type :4;
        uint8_t id :10;
        uint16_t data :10;
} kilobot_message;

#define uint8_t_LENGTH 10 // in bits
#define KILOBOT_MESSAGE_TYPE_LENGTH 4 // in bits
#define KILOBOT_MESSAGE_DATA_LENGTH 10 // in bits
#define KILOBOT_MAX_COLOUR 255 // colour channel
#define KILOBOT_REFRESH_DELAY 3 // in milliseconds

#define KILOBOT_MAX_ID 1024 //

class KilobotEnvironment;

#include <QObject>

struct kilobot_colour
{
    kilobot_channel_colour r;
    kilobot_channel_colour g;
    kilobot_channel_colour b;
};

class Kilobot : public QObject {
    Q_OBJECT
public:
    Kilobot(uint8_t identifier, kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment *environment);
    Kilobot() {}
    ~Kilobot();
    // copy constructor
    Kilobot(const Kilobot& other);
    uint8_t getID();
    void setID(uint8_t);
    kilobot_pos getXPosition();
    kilobot_pos getYPosition();
    kilobot_colour getLedColour();
    //kilobot_colour resolveKilobotState(stateColours);
    void updateState(kilobot_pos xPosition, kilobot_pos yPosition, kilobot_colour colourValues, KilobotEnvironment * environment);

    /*!
     * \brief updateHardware
     * Copy the Kilobot (for thread safety) and signal
     * the environment to update the hardware using that
     * copy
     */
    void updateHardware();

    /*!
     * \brief updateExperiment
     * Copy the Kilobot (for thread safety) and signal
     * the experiment to update the hardware using that
     * copy, as well as a pointer for remapping signal /
     * slot connections (should NOT be de-referenced)
     */
    void updateExperiment();

    int lightThreshold = 230;
    int vsensorValue = 0;

signals:
    void sendUpdateToHardware(Kilobot);
    void sendUpdateToExperiment(Kilobot*,Kilobot);

private:
    uint8_t id = 0;
    kilobot_pos x = 0;
    kilobot_pos y = 0;
    kilobot_colour col = {0,0,0};
    KilobotEnvironment * env = NULL;
};

#endif // KILOBOT_H
