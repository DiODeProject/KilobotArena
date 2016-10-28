#ifndef CALIBRATEARENA_H
#define CALIBRATEARENA_H
#include <ios>
#include <vector>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/videoio.hpp>


// allow easy addressing of OpenCV functions
using namespace cv;
using namespace std;

// Qt base include
#include <QObject>
#include <QPoint>
#include <QPixmap>
// for thread buffer communication
#include <QSemaphore>
#include <QTimer>
#include <QElapsedTimer>

#include "kilobot.h"

// buffers and semaphores
struct srcBuffer {
    UMat warped_image;
    UMat warped_mask;
    Point corner;
    Size size;
    UMat full_warped_image;
};

#define BUFF_SIZE 5

enum srcDataType {
    IMAGES,
    VIDEO,
    CAMERA
};

enum trackerType {
    CIRCLES_NAIVE,
    CIRCLES_LOCAL,
    PARTICLE_FILTER
};

class acquireThread;

/*!
 * \brief The KilobotTracker class
 *
 * This class contains the code that tracks Kilobots in the live camera or offline video feeds.
 *
 */
class KilobotTracker : public QObject
{
    Q_OBJECT
public:
    explicit KilobotTracker(QPoint smallImageSize = QPoint(300,300), QObject *parent = 0);
    ~KilobotTracker();

signals:
    /*!
     * \brief errorMessage
     * Qt signal to update the UI message QLabel
     */
    void errorMessage(QString);

    void setStitchedImage(QPixmap);

public slots:
    /*!
     * \brief startLoop
     * This slot is the target of the timeout on the QTimer tick, and fetches warped images from the thread buffers and
     * stitches them
     */
    void startLoop();

    /*!
     * \brief stitchImages
     * Use the existing feature matches to stitch the images
     */
    void stitchImages();

    /*!
     * \brief loadCalibration
     * Load the calibration matrices from an OpenCV FileStorage format
     */
    void loadCalibration();

    /*!
     * \brief findKilobots
     * Find the locations of Kilobots in the stitched image
     */
    void findKilobots();

    void setCamOrder();

    void setKbMin(int);
    void setKbMax(int);
    void setCannyThresh(int);
    void setHoughAcc(int);

private:

    void trackKilobots();

    Mat finalImage;
    Mat finalImageR;
    Mat finalImageG;
    Mat finalImageB;

    vector < UMat > warpedImages;
    vector < UMat > warpedMasks;
    vector < Point > corners;
    vector < Size > sizes;

    vector < Mat > Ks;
    vector < Mat > Rs;
    Point2f arenaCorners[4];
    bool haveCalibration = false;
    QTimer tick;

    acquireThread * threads[4] = {NULL,NULL,NULL,NULL};

    int time = 0;

    /*!
     * \brief smallImageSize
     * Assigned in the constructor
     */
    QPoint smallImageSize;

    Ptr<detail::ExposureCompensator> compensator;

    QElapsedTimer timer;

    bool loadFirstIm = false;

    int kbMinSize = 7;
    int kbMaxSize = 11;
    int houghAcc = 120;
    int cannyThresh = 15;

    MultiTracker * tracker = NULL;

    trackerType trackType = CIRCLES_LOCAL;

    QVector < Kilobot > kilos;

    QVector < QVector < int > > exclusionTestsIndices;

    float last_time = 0.0f;

};



#endif // CALIBRATEARENA_H
