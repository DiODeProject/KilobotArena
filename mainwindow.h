#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <ios>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>
using namespace cv;

#include <QMainWindow>

// Project includes
#include "kilobottracker.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:


    /*!
     * \brief setVideoSource
     * Method to generate a dialog used for setting the video source
     */
    void setVideoSource();

private:
    Ui::MainWindow *ui;

    KilobotTracker kbtracker;
};

#endif // MAINWINDOW_H
