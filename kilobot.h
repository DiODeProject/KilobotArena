/*
 * Kilobot.h
 *
 *  Created on: 16 Sep 2016
 *      Author: marshall
 */

#ifndef KILOBOT_H_
#define KILOBOT_H_

#include <stdint.h>
#include <QPointF>
#include <QVector>

enum lightColour {
    OFF,
    RED,
    GREEN,
    BLUE
};

typedef uint16_t kilobot_id;
typedef unsigned char kilobot_channel_colour;
//typedef unsigned char kilobot_message_type;
typedef struct {
        uint8_t type :4;
        kilobot_id id :10;
        uint16_t data :10;
} kilobot_message;

struct kilobot_broadcast {
        uint8_t type;
        QVector < uint8_t > data;
};

#define uint8_t_LENGTH 10 // in bits
#define KILOBOT_MESSAGE_TYPE_LENGTH 4 // in bits
#define KILOBOT_MESSAGE_DATA_LENGTH 10 // in bits
#define KILOBOT_MAX_COLOUR 255 // colour channel
#define KILOBOT_REFRESH_DELAY 3 // in milliseconds

#define KILOBOT_MAX_ID 1024 //

#define UNASSIGNED_ID INT16_MAX

/*enum tracking_type {
    POS_ONLY,
    LED_ONLY,
    ADAPTIVE_LED_ONLY,
    POS_LED,
    POS_ADAPTIVE_LED,
};*/

enum tracking_flags {
    POS = 1 << 0,
    LED = 1 << 1,
    ADAPTIVE_LED = 1 << 2,
    ROT = 1 << 3
};


#include <QObject>

/*struct kilobot_colour
{
    kilobot_channel_colour r;
    kilobot_channel_colour g;
    kilobot_channel_colour b;
};*/
typedef lightColour kilobot_colour;


class ColourBuffer{
public:
    ColourBuffer() : ColourBuffer(1) {}
    ~ColourBuffer() {}
    ColourBuffer(int size);
    void addColour(lightColour newColour);
    lightColour getAvgColour();
    lightColour getLastColour() {return buffer.at(buffer.size()-1);}

private:
    int buffer_size;
    QVector < lightColour > buffer;

};

class OrientationBuffer{
public:
    OrientationBuffer() : OrientationBuffer(1) {}
    ~OrientationBuffer() {}
    OrientationBuffer(int size) : buffer_size(size) {}
    void addOrientation(QPointF newOrientation);
    QPointF getAvgOrientation();
    QPointF getLastOrientation() {return buffer.at(buffer.size()-1);}

private:
    int buffer_size;
    QVector < QPointF > buffer;

};

class PositionBuffer{
public:
    PositionBuffer() : PositionBuffer(1) {}
    ~PositionBuffer() {}
    PositionBuffer(int size) : buffer_size(size) {}
    void addPosition(QPointF newPosition);
    QPointF getOrientationFromPositions();
    QPointF getLastPosition() {return buffer.at(buffer.size()-1);}

private:
    int buffer_size;
    QVector < QPointF > buffer;

};

class Kilobot : public QObject {
    Q_OBJECT
public:
    Kilobot(kilobot_id identifier, QPointF position, QPointF velocity, kilobot_colour colourValues);
    Kilobot() {}
    ~Kilobot();
    // copy constructor
    Kilobot(const Kilobot& other);
    kilobot_id getID();
    void setID(kilobot_id);
    QPointF getPosition();
    QPointF getVelocity();
    kilobot_colour getLedColour();
    //kilobot_colour resolveKilobotState(stateColours);
    void updateState(QPointF position, QPointF velocity, kilobot_colour colourValues);

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

    ColourBuffer colBuffer = ColourBuffer(5);
    OrientationBuffer velocityBuffer = OrientationBuffer(5);
    PositionBuffer posBuffer = PositionBuffer(6);

signals:
    void sendUpdateToHardware(Kilobot);
    void sendUpdateToExperiment(Kilobot*,Kilobot);

private:
    kilobot_id id = UNASSIGNED_ID;
    QPointF pos = QPointF(0,0);
    QPointF vel = QPointF(1,1);
    kilobot_colour col = OFF;

};

#endif // KILOBOT_H
