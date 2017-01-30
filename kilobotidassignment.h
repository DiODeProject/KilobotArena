#ifndef KILOBOTIDASSIGNMENT_H
#define KILOBOTIDASSIGNMENT_H

#include "kilobotexperiment.h"
#include <QElapsedTimer>

enum assignStage {
    START,
    SEND,
    TEST,
    RETRY,
    COMPLETE,
};

const int baseFourMultipliers[6] = {1,4,16,64,256,1024};
const int binaryMultipliers[11] = {1,2,4,8,16,32,64,128,256,512,1024};


//#define ASSIGNED -INT16_MAX
#define DUPE UINT16_MAX

class KilobotIDAssignment : public KilobotExperiment
{
    Q_OBJECT
public:
    KilobotIDAssignment();
    virtual ~KilobotIDAssignment() {}

public slots:
        void initialise(bool);
        void run();

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
};

#endif // KILOBOTIDASSIGNMENT_H
