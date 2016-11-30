#ifndef KILOBOTEXPERIMENT_H
#define KILOBOTEXPERIMENT_H

#include <QObject>
#include <QDebug>
#include <kilobotenvironment.h>
#include <kilobottracker.h>
#include <kilobotoverheadcontroller.h>

class KilobotExperiment : public QObject
{
    Q_OBJECT
public:
    explicit KilobotExperiment(QObject *parent = 0);

signals:
    void updateKilobotStates();
    void getInitialKilobotStates();
    void experimentComplete();
    void saveImage(QString);

public slots:
    virtual void initialise(bool) = 0;
    virtual void run() = 0;

    /*!
     * \brief updateStateRequiredCode
     * \param kilobot
     * \param kilobotCopy
     *
     * Slot that makes sure that some code is run BEFORE the derived function
     */
    void updateStateRequiredCode(Kilobot* kilobot, Kilobot kilobotCopy)
    {
        //qDebug() << "pre set state 2";
        // store pointer for connecting
        this->currKilobot = kilobot;
        updateKilobotEnvironment(kilobotCopy);
    }

    /*!
     * \brief setupInitialStateRequiredCode
     * \param kilobot
     * \param kilobotCopy
     *
     * Slot that makes sure that some code is run BEFORE the derived function
     */
    void setupInitialStateRequiredCode(Kilobot* kilobot, Kilobot kilobotCopy)
    {
        //qDebug() << "pre set state";
        // store pointer for connecting
        this->currKilobot = kilobot;
        // switch the signal from setup to standard
        kilobot->disconnect(SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)));
        connect(kilobot,SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)), this, SLOT(updateStateRequiredCode(Kilobot*,Kilobot)));
        setupInitialKilobotEnvironment(kilobotCopy);
    }

protected:
    double time;

    QVector <KilobotEnvironment *> environments;

    void setCurrentKilobotEnvironment(KilobotEnvironment * environment) {
        if (currKilobot != NULL && environment != NULL) {
            QObject::disconnect(currKilobot,SIGNAL(sendUpdateToHardware(Kilobot)), 0, 0);
            QObject::connect(currKilobot,SIGNAL(sendUpdateToHardware(Kilobot)), environment, SLOT(generateEnvironment(Kilobot)));
        }
    }

    virtual void updateKilobotEnvironment(Kilobot kilobotCopy) = 0; // provided in derived class to implement experiment logic for Kilobot state updates
    virtual void setupInitialKilobotEnvironment(Kilobot kilobotCopy) = 0;

private:
    Kilobot * currKilobot = NULL;

};

#endif // KILOBOTEXPERIMENT_H
