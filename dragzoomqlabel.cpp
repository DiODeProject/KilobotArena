#include "dragzoomqlabel.h"
#include <QMouseEvent>
#include <QDebug>

dragZoomQLabel::dragZoomQLabel(QWidget *parent) : QLabel(parent)
{

}

void dragZoomQLabel::mousePressEvent(QMouseEvent *ev)
{

    if (ev->button() == Qt::LeftButton) {
        this->isDragged = true;
        this->setMouseTracking(true);
        emit moving(QPoint(ev->localPos().x(),ev->localPos().y()));
        ev->accept();
    }

}

void dragZoomQLabel::mouseMoveEvent(QMouseEvent *ev)
{

    if (this->isDragged) {
        emit moving(QPoint(ev->localPos().x(),ev->localPos().y()));
    }
    ev->accept();

}

void dragZoomQLabel::mouseReleaseEvent(QMouseEvent *ev)
{

    if (ev->button() == Qt::LeftButton) {
        this->isDragged = false;
        emit this->moveDone();
        this->setMouseTracking(false);
        ev->accept();
    }

}

