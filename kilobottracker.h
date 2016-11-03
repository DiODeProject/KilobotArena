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
    Mat full_warped_image;
};

#define BUFF_SIZE 2

#define IM_WIDTH 2048
#define IM_HEIGHT 1536


enum srcDataType {
    IMAGES,
    VIDEO,
    CAMERA
};

enum trackerType {
    CIRCLES_NAIVE,
    CIRCLES_LOCAL,
    PARTICLE_FILTER,
    SAMPLE_MATCH
};

enum lightColour {
    OFF,
    BLUE,
    GREEN,
    RED
};

struct kiloLight {
    lightColour col;
    Point pos;
};

struct circlesLocalTrackerData {
    // mappings from the image indices to the quadrants
    int inds[4];
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
     * \brief iterateTracker
     * Use the existing feature matches to stitch the images and track the kilobots
     */
    void iterateTracker();

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

    /*!
     * \brief setCamOrder
     * If the camera order does not match the calibration order, alter
     */
    void setCamOrder();

    // accessors - docs not required??
    void setSourceType(bool);
    void setKbMin(int);
    void setKbMax(int);
    void setCannyThresh(int);
    void setHoughAcc(int);

    /*!
     * \brief setVideoDir
     * \param dir
     * Set the path to video files for tracking
     */
    void setVideoDir(QString dir);

private:

    // PRIVATE METHODS

    /*!
     * \brief trackKilobots
     * The method used to contain the tracking algorithm for one timestep
     */
    void trackKilobots();

    /*!
     * \brief setupStitcher
     * Setup required after loading the calibration data
     */
    void setupStitcher();

    /*!
     * \brief showMat
     * Convert a Mat for display and send it as a QPixmap via the setStichedImage signal
     */
    void showMat(Mat &);

    /*!
     * \brief getKiloBotLight
     * \param channels
     * \param centreOfBox
     * \return
     * Used to detect the presence, colour, and position of a kiloBot's light and return it
     */
    kiloLight getKiloBotLight(Mat channels[3], Point centreOfBox);

    /*!
     * \brief launchThreads
     * Launches the threads for each of the source images
     */
    void launchThreads();


    // INTERNAL VARIABLES

    Mat finalImage;

    Mat fullImages[4][3];

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
    Ptr<detail::Blender> blender;

    QElapsedTimer timer;

    bool loadFirstIm = false;

    int kbMinSize = 10;
    int kbMaxSize = 31;
    int houghAcc = 25;
    int cannyThresh = 50;

    srcDataType srcType = CAMERA;
    QString videoPath;

    MultiTracker * tracker = NULL;

    trackerType trackType = CIRCLES_LOCAL;

    QVector < Kilobot > kilos;

    QVector < float > kiloHeadings;

    QVector < Mat > samples;

    QVector < QVector < int > > exclusionTestsIndices;

    float last_time = 0.0f;

    circlesLocalTrackerData clData;

    Size fullSize;
    Point fullCorner;

};



#endif // CALIBRATEARENA_H
