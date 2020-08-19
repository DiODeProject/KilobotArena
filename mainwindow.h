#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <ios>

#ifndef USE_OPENCV3
// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videostab/videostab.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>

#else

// OpenCV 3:
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>

#endif

using namespace cv;

#include <QMainWindow>

// Project includes
#include "kilobottracker.h"
#include "kilobotoverheadcontroller.h"
//#include "kilobotexperiment.h"

class UserThread;

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

    void getExperiment();

    void assignIDs();

    void calibrate();

    void identify();

    void toggleRunButton(int expTypeInt);

    void activateExpButtons(int swarmSize);

    void rotate_pos();
    void rotate_neg();

    void runExpt();

    void setGUILayout(QWidget *);

    void left();
    void right();
    void straight();

    void test_id();

private:
    Ui::MainWindow *ui;

    KilobotTracker kbtracker;
    KilobotOverheadController ohc;
    UserThread * thread = NULL;

    int robcomm[3] = {3,3,3};

    QString userExpt;
};

#endif // MAINWINDOW_H
