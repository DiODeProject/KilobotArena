/*!
 * Kilobottracker.h
 *
 *  Created on: 3 Oct 2016
 *  Author: Alex Cope
 */

#ifndef CALIBRATEARENA_H
#define CALIBRATEARENA_H
#include <ios>
#include <vector>

#define USE_CUDA true



#ifndef USE_OPENCV3

// OpenCV 2 includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videostab/videostab.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/opencv.hpp>
//GPU stuff
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>

#ifdef USE_CUDA
    #define MAT_TYPE cuda::GpuMat
    #define CV_NS cv::cuda::
#else
    #define MAT_TYPE Mat
    #define CV_NS cv::
#endif

#else

using namespace std;

// OpenCV 3 :
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv2/tracking.hpp>
//#include <opencv2/tracking/tracker.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudawarping.hpp>

#ifdef USE_CUDA
    #define MAT_TYPE cuda::GpuMat
    #define CV_NS cv::cuda::
#else
    #define MAT_TYPE UMat
    #define CV_NS cv::
#endif

#endif

// allow easy addressing of OpenCV functions
using namespace cv;




// Qt base include
#include <QObject>
#include <QPoint>
#include <QPixmap>
// for thread buffer communication
#include <QSemaphore>
#include <QTimer>
#include <QElapsedTimer>
#include <QTime>
//#include <QColor>

#include "kilobot.h"

// buffers and semaphores
struct srcBuffer {
    MAT_TYPE warped_image;
    MAT_TYPE warped_mask;
    Point corner;
    Size size;
    MAT_TYPE full_warped_image;
};

#define BUFF_SIZE 2

#define IM_WIDTH 2048
#define IM_HEIGHT 1536

#define IDENTIFY_TIMEOUT 10


enum srcDataType {
    IMAGES,
    VIDEO,
    CAMERA
};

enum trackerType {
    CIRCLES_NAIVE,
    CIRCLES_LOCAL,
    MY_HAPPY_OTHER_TRACKER,
};

struct indexPair {
    int a;
    int b;
};

struct kiloLight {
    lightColour col;
    Point pos;
};

enum stageType {
    IDENTIFY,
    TRACK
};


struct circlesLocalTrackerData {
    // mappings from the image indices to the quadrants
    int inds[4];
};

// DRAWABLES:

struct drawnCircle {
    Point pos;
    int r;
    QColor col;
};

class acquireThread;
class KilobotExperiment;

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

    KilobotExperiment * expt;


signals:
    /*!
     * \brief errorMessage
     * Qt signal to update the UI message QLabel
     */
    void errorMessage(QString);

    void setStitchedImage(QPixmap);

    void identifyKilo(uint8_t);

    void broadcastMessage(kilobot_broadcast);

    void startExperiment(bool);

    void stopExperiment();

    void testtesttest(Kilobot*,Kilobot);

public slots:
    /*!
     * \brief startLoop
     * This slot is the target of the timeout on the QTimer tick, and fetches warped images from the thread buffers and
     * stitches them
     */
    void LOOPstartstop(int stage);

    /*!
     * \brief iterateTracker
     * Use the existing feature matches to stitch the images and track the kilobots
     */
    void LOOPiterate();

    /*!
     * \brief loadCalibration
     * Load the calibration matrices from an OpenCV FileStorage format
     */
    void SETUPloadCalibration();

    /*!
     * \brief findKilobots
     * Find the locations of Kilobots in the stitched image
     */
    void SETUPfindKilobots();

    /*!
     * \brief identifyKilobots
     * Find out what IDs the Kilobots have
     */
    void identifyKilobots();

    /*!
     * \brief setCamOrder
     * If the camera order does not match the calibration order, alter
     */
    void SETUPsetCamOrder();

    // drawing slots
    void drawCircle(QPointF pos, float r, QColor col) {

        int r_int = r;
        this->circsToDraw.push_back(drawnCircle {Point(pos.x(),pos.y()), r_int, col});

    }

    void clearDrawings() {

        this->circsToDraw.clear();

    }

    void saveImage(QString file) {
        if (!finalImageCol.empty()) {
            cv::imwrite(file.toStdString(),this->finalImageCol);
        }
    }


    // accessors - docs not required??
    void setSourceType(bool val) {if (val) this->srcType = CAMERA; else this->srcType = VIDEO;}
    void setKbMin(int val){this->kbMinSize = val;}
    void setKbMax(int val) {this->kbMaxSize = val;}
    void setCannyThresh(int val) {this->cannyThresh = val;}
    void setHoughAcc(int val) {this->houghAcc = val;}

    /*!
     * \brief setVideoDir
     * \param dir
     * Set the path to video files for tracking
     */
    void setVideoDir(QString dir) {this->videoPath = dir;}

    void updateKilobotStates();

    void getInitialKilobotStates();

    void setTrackingType(int t_type) {this->t_type = t_type;}

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
    void SETUPstitcher();

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

#ifdef USE_CUDA
    kiloLight getKiloBotLight(cuda::GpuMat  channels[3], Point centreOfBox, int index);
    kiloLight getKiloBotLightAdaptive(cuda::GpuMat  channels[3], Point centreOfBox, int index);
#else
    kiloLight getKiloBotLight(Mat channels[3], Point centreOfBox, int index);
    kiloLight getKiloBotLightAdaptive(Mat channels[3], Point centreOfBox, int index);
#endif

    /*!
     * \brief getKiloBotBoundingBox
     * \param index
     * \param scale
     * \return
     *
     * For a single Kilobot obtain an openCV Rect of the bounding box around the kilobot
     */
    Rect getKiloBotBoundingBox(int index, float scale);

    /*!
     * \brief launchThreads
     * Launches the threads for each of the source images
     */
    void THREADSlaunch();

    void THREADSstop();

    void identifyKilobot(int);

    void drawOverlay(Mat &);


    // INTERNAL VARIABLES

    int t_type = POS | ADAPTIVE_LED | ROT;



    Mat finalImageCol;

#ifdef USE_CUDA
    cuda::GpuMat finalImage;
    cuda::GpuMat fullImages[4][3];
    // make thread safe
    cuda::Stream stream;
#else
    Mat finalImage;
    Mat fullImages[4][3];
#endif

    vector < MAT_TYPE > warpedImages;
    vector < MAT_TYPE > warpedMasks;
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

    trackerType trackType = CIRCLES_NAIVE;//CIRCLES_LOCAL;

    QVector < Kilobot * > kilos;

    QVector < float > kiloHeadings;

    QVector < QPointF > kiloOldPos;

    QVector < QVector < int > > exclusionTestsIndices;

    float last_time = 0.0f;

    circlesLocalTrackerData clData;

    Size fullSize;
    Point fullCorner;

    uint currentID = 0;
    uint found = IDENTIFY_TIMEOUT;

    stageType stage = TRACK;

    Mat testAdap;

    QVector < drawnCircle > circsToDraw;

#ifdef USE_CUDA
    Ptr<cuda::HoughCirclesDetector> hough;
    cuda::GpuMat kbLocs;
#endif

};



#endif // CALIBRATEARENA_H
