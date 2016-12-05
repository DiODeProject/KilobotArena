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

void dragZoomQLabel::drawCircle(int x_cen, int y_cen, int rad, QPaintEvent *)
{


    //Draw Circle:
    //drawArc(x_cen,y_cen,x_rad,y_rad,angle start (1/16th of degree), angle end (1/16th of degree))
//    QPainter painter(this);

//    painter.setBrush(QBrush(Qt::black));
//    painter.drawArc(x_cen,y_cen,150,50,0,16*360);
//    item = new QGraphicsPixmapItem(QPixmap::fromImage(worldImage));
//    this->addItem(item);

//    QPainter painter(this);
//    painter.setPen(Qt::blue);
//    painter.setFont(QFont("Arial", 30));
//    painter.drawText(rect(), Qt::AlignCenter, "Qt");

//    painter.drawArc(x_cen,y_cen,rad,rad,0,16*360);


    //    QRectF rectangle(10.0, 20.0, 80.0, 60.0);
    //    int startAngle = 30 * 16;
    //    int spanAngle = 120 * 16;

    //    QPainter painter(this);
    //    painter.drawArc(rectangle, startAngle, spanAngle);

}



