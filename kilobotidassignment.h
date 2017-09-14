#ifndef KILOBOTIDASSIGNMENT_H
#define KILOBOTIDASSIGNMENT_H

#include "kilobotexperiment.h"
#include <QElapsedTimer>
#include <QFile>
#include "kilobot.h"

enum assignStage {
    START,
    SEND,
    TEST,
    RETRY,
    COMPLETE,
};

const int baseFourMultipliers[6] = {1,4,16,64,256,1024};
const int binaryMultipliers[11] = {1,2,4,8,16,32,64,128,256,512,1024};

//static const double MM_TO_PIXEL = 2000.0/2200.0;
//static const double PIXEL_TO_MM = 2200.0/2000.0;
static const double MM_TO_PIXEL = 2000.0/2000.0;
static const double PIXEL_TO_MM = 2000.0/2000.0;

//#define ASSIGNED -INT16_MAX
#define DUPE UINT16_MAX

class KiloLog {
public:
    // constructors
    KiloLog() {}
    KiloLog(kilobot_id id, QPointF pos, double rot, kilobot_colour col) :
        id(id), position(pos), orientation(rot), colour(col) {
        this->digits.resize(11);
        this->digits.fill(-1);
    }

    // methods
    void updateAllValues(kilobot_id id, QPointF pos, double rot, kilobot_colour col){
        this->id = id;
        this->position = pos;
        this->orientation = rot;
        this->colour = col;
    }
    void setPos(QPointF pos){
        this->position = pos;
    }
    void setOrientation(double rot){
        this->orientation = rot;
    }
    void setCol(kilobot_colour col){
        this->colour = col;
    }

    // variables
    kilobot_id id;
    QPointF position;
    double orientation;
    kilobot_colour colour;
    QVector <int> digits;
};

class KilobotIDAssignment : public KilobotExperiment
{
    Q_OBJECT
public:
    KilobotIDAssignment();
    virtual ~KilobotIDAssignment() {}

public slots:
        void initialise(bool);
        void run();
        void stopExperiment();

private:
        void updateKilobotState(Kilobot kilobotCopy);
        void setupInitialKilobotState(Kilobot kilobotCopy);

// internal vars
        QVector < uint16_t > tempIDs;
        QVector < bool > isAssigned;
        assignStage stage;
        float lastTime = 0.0f;
        //int numFound = 0;
        bool dupesFound = 0;
        int numSegments = 0;

        bool switchSegment = true;
        QElapsedTimer t;
        int t_since;

        // log variables
        bool saveImages = false;
        int savedImagesCounter = 0;
        QFile log_file;
        QString log_filename_prefix="log_id_ass";
        QTextStream log_stream;
        //QVector < kilobot_id >  allKiloIDs;
        QVector <KiloLog> allKilos;
        bool updatedCol = false;

};

#endif // KILOBOTIDASSIGNMENT_H
