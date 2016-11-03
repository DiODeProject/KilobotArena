#include "kilobotoverheadcontroller.h"


KilobotOverheadController::KilobotOverheadController(QObject *parent) : QObject(parent)
{
    // OHC link open
}

KilobotOverheadController::~KilobotOverheadController()
{
    // OHC link close
}

void KilobotOverheadController::identifyKilobot(kilobot_id id)
{
    assert(id <= pow(2, KILOBOT_ID_LENGTH) - 1);
}

void KilobotOverheadController::signalKilobot(kilobot_id id, kilobot_message_type message, kilobot_message_data data)
{
    assert(id <= pow(2, KILOBOT_ID_LENGTH) - 1);
    assert(message <= pow(2, KILOBOT_MESSAGE_TYPE_LENGTH) - 1);
    assert(data <= pow(2, KILOBOT_MESSAGE_DATA_LENGTH) - 1);

    // TODO this method should work on a queue basis - signals should be queued until at least 3 are available, then broadcast in a single message
}
