#include "clicksignalqlabel.h"
#include <QMouseEvent>

clickSignalQLabel::clickSignalQLabel(QWidget *parent)  : QLabel(parent)
{
}

void clickSignalQLabel::mousePressEvent(QMouseEvent *ev)
{

    if (ev->button() == Qt::LeftButton) {
        emit clicked(QPoint(ev->localPos().x(),ev->localPos().y()));
        ev->accept();
    }

}
