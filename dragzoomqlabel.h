#ifndef DRAGZOOMQLABEL_H
#define DRAGZOOMQLABEL_H

#include <QLabel>

class dragZoomQLabel : public QLabel
{
    Q_OBJECT

public:
    dragZoomQLabel(QWidget *parent = 0);

protected slots:
    void mousePressEvent(QMouseEvent *ev);
    void mouseMoveEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);

signals:
    void moving(QPoint);
    void moveDone();

private:
    bool isDragged = false;

};

#endif // DRAGZOOMQLABEL_H
