/*!
 * Kilobottracker.cpp
 *
 *  Created on: 3 Oct 2016
 *  Author: Alex Cope
 */

#include "kilobottracker.h"
#include "kilobotexperiment.h"
#include <QImage>
#include <QDebug>
#include <QThread>
#include <QLineEdit>
#include <QDir>
#include <QSettings>
#include <QFileDialog>
#include <QtMath>
//#include <opencv2/gpu/gpMAT_TYPE.hpp>

//#define TEST_WITHOUT_CAMERAS

QSemaphore srcFree[4];
QSemaphore srcUsed[4];
srcBuffer srcBuff[4][BUFF_SIZE];
QSemaphore srcStop[4];
QSemaphore camUsage;

int camOrder[4] = {0,1,2,3};

/*!
 * \brief The acquireThread class
 * This thread acquires input data from a source, applies the local warps
 * to the data, and then places the data in a circular buffer for use by
 * the main thread, which operates on a QTimer to allow UI responsivity
 */
class acquireThread : public QThread
{
public:

    ~acquireThread() {
        // shut down cam
        camUsage.acquire();
        if (cap.isOpened()) cap.release();
        camUsage.release();
    }

    // reprojection details
    Mat K;
    Mat R;
    Point corner;
    Size size;
    Size fullSize;
    Point fullCorner;
    Point2f arenaCorners[4];
    QString videoDir = "";

    // the index of the source
    uint index = 0;

    bool keepRunning = true;

    srcDataType type = CAMERA;

    cv::VideoCapture cap;

private:
    /*!
     * \brief run
     * The execution method for the thread, performing the stitching process
     */
    void run() {

        QThread::currentThread()->setPriority(QThread::TimeCriticalPriority);

        #ifdef USE_OPENCV3
        if (ocl::haveOpenCL()) {
            ocl::setUseOpenCL(true);
        }
        #endif

        uint time = 0;
        Mat image;
        Mat mask;

        Ptr<WarperCreator> warper_creator;
        warper_creator = new cv::PlaneWarper();//makePtr<cv::PlaneWarper>();
        Ptr<detail::RotationWarper> warper = warper_creator->create(2000.0f);

        Point2f outputQuad[4];
        outputQuad[0] = Point(0,0);
        outputQuad[1] = Point(2000,0);
        outputQuad[2] = Point(0,2000);
        outputQuad[3] = Point(2000,2000);

        // loop
        while (keepRunning) {

            // check for stop signal
            if (srcStop[index].available()) {
                keepRunning = false;
            }

            if (srcFree[index].available()) {

                // get data
                if (type == IMAGES) {
                    image = imread((this->videoDir+QDir::toNativeSeparators("/")+QString("frame_00200_")+QString::number(index)+QString(".jpg")).toStdString());
                }
                else if (type == CAMERA) {
#ifndef TEST_WITHOUT_CAMERAS
                    camUsage.acquire();
                    if (!cap.isOpened() /*&& camOrder[index]!=3  TEMPORARY!!! */) {
                        cap.open(camOrder[index]);
                        // set REZ
                        if (cap.isOpened()) {
                            cap.set(CV_CAP_PROP_FRAME_WIDTH, IM_WIDTH);
                            cap.set(CV_CAP_PROP_FRAME_HEIGHT, IM_HEIGHT);
                        } else {
                            this->keepRunning = false;
                            continue;
                        }
                    }
                    //qDebug() << "Thread" << index << time << QTime::currentTime();
                    if (cap.isOpened()) {
                        // exhaust buffer
                        cap.grab();
                        cap.grab();
                        camUsage.release();

                        camUsage.acquire();
                        cap.retrieve(image);
                        camUsage.release();
                    }
                    else
#endif
                    {
                        image = Mat(IM_HEIGHT,IM_WIDTH, CV_8UC3, Scalar(0,0,0)); /* TEMPORARY!!! */
                        camUsage.release();
                    }

                } else if (type == VIDEO) {
                    image = imread((QString("/home/alex/Downloads/study-table-bias-r1/frame_%1_%2").arg(time+1, 5,10, QChar('0')).arg(index)+QString(".jpg")).toStdString());
                    if (image.empty()) continue;
                }

                // Prepare images masks
                if (mask.size().width < 10) {
                  mask.create(image.size(), CV_8U);
                }
                mask.setTo(Scalar::all(255));

                // check semaphore
                srcFree[index].acquire();

                srcBuff[index][time % BUFF_SIZE].corner = warper->warp(image, K, R, INTER_LINEAR, BORDER_REFLECT, srcBuff[index][time % BUFF_SIZE].warped_image);
                srcBuff[index][time % BUFF_SIZE].size = srcBuff[index][time % BUFF_SIZE].warped_image.size();
                warper->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, srcBuff[index][time % BUFF_SIZE].warped_mask);

                // only do this if we are not loading calibration
                if (!(this->corner.x == -1 && this->corner.y == -1)) {

#ifdef SLOW_WARP
                    // create full image
                    MAT_TYPE temp(fullSize.height,fullSize.width, CV_8UC3, Scalar(0,0,0));
                    MAT_TYPE temp2;
                    cv::resize(srcBuff[index][time % BUFF_SIZE].warped_image, temp2, Size(size.width-10,size.height-10));
                    temp2.copyTo(temp(Rect(corner.x-fullCorner.x,corner.y-fullCorner.y,size.width-10, size.height-10)));

                    cv::resize(temp, temp2,Size(1536,1536));

                    Mat M = getPerspectiveTransform(this->arenaCorners,outputQuad);

                    warpPerspective(temp2, srcBuff[index][time % BUFF_SIZE].full_warped_image, M, Size(2000,2000));
#else
                    // test without big Mats
#define ADJ 10 // adjustment used to compensate for placing calibration images on table, and not at kilobot height
                    MAT_TYPE temp2;
#ifdef ADJ
                    MAT_TYPE temp;
                    cv::resize(srcBuff[index][time % BUFF_SIZE].warped_image, temp, Size(size.width-ADJ,size.height-ADJ));
                    cv::resize(temp, temp2,Size((1536*(size.width-ADJ))/fullSize.width,(1536*(size.height-ADJ))/fullSize.height));
#else
                    cv::resize(srcBuff[index][time % BUFF_SIZE].warped_image, temp2,Size((1536*(size.width-ADJ))/fullSize.width,(1536*(size.height-ADJ))/fullSize.height));
#endif

                    Point2f arenaCorners_adj[4];
                    Point2f outputQuad_adj[4];
                    for (int i = 0; i < 4; ++i) {
                        arenaCorners_adj[i] = arenaCorners[i] - Point2f(((corner.x-fullCorner.x)*1536)/fullSize.width,((corner.y-fullCorner.y)*1536)/fullSize.height);
                        outputQuad_adj[i] = outputQuad[i] - Point2f((corner.x>300)*1000, (corner.y-fullCorner.y>300)*1000);
                    }

                    Mat M = getPerspectiveTransform(arenaCorners_adj,outputQuad_adj);

                    warpPerspective(temp2, srcBuff[index][time % BUFF_SIZE].full_warped_image, M, Size(1100,1100));
#endif
                }

                srcUsed[index].release();

                ++time;
            }

        }
        QThread::currentThread()->setPriority(QThread::NormalPriority);

    }
};


KilobotTracker::KilobotTracker(QPoint smallImageSize, QObject *parent) : QObject(parent)
{
    this->smallImageSize = smallImageSize;
    this->tick.setInterval(1);
    connect(&this->tick, SIGNAL(timeout()), this, SLOT(LOOPiterate()));

    // initialise semaphores
    srcFree[0].release(BUFF_SIZE);
    srcFree[1].release(BUFF_SIZE);
    srcFree[2].release(BUFF_SIZE);
    srcFree[3].release(BUFF_SIZE);

    camUsage.release(1);

}

KilobotTracker::~KilobotTracker()
{
    if (this->threads[0] && this->threads[0]->isRunning()) {
        this->THREADSstop();
    }

    // clean up memory
    for (uint i = 0; i < 4; ++i) {
        if (this->threads[i]) {
            delete this->threads[i];
        }
    }
}

void KilobotTracker::LOOPstartstop(int stage)
{

    this->stage = (stageType) stage;

    // check if running
    if (this->threads[0] && this->threads[0]->isRunning()) {

        // reset IDing
        //this->aStage = START;
        currentID = 0;

        emit errorMessage(QString("FPS = ") + QString::number(float(time)/(float(timer.elapsed())/1000.0f)));
        this->THREADSstop();

        // Stop the experiment
        emit stopExperiment();

        QThread::currentThread()->setPriority(QThread::NormalPriority);

        return;

    }

    QThread::currentThread()->setPriority(QThread::TimeCriticalPriority);

    // only if we have calib data
    if (!this->haveCalibration) {
        return;
    }

    // launch threads
    this->THREADSlaunch();

    // connect kilobots
    for (int i = 0; i < this->kilos.size(); ++i) {
        disconnect(kilos[i]);
        connect(this->kilos[i],SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)), this->expt, SLOT(setupInitialStateRequiredCode(Kilobot*,Kilobot)));
    }

    if (!this->compensator) {
        // calculate to compensate for exposure
        compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);
    }

    if (!this->blender) {
        // blend the images
        blender = detail::Blender::createDefault(detail::Blender::FEATHER, true);
    }

    this->warpedImages.resize(4);
    this->warpedMasks.resize(4);
    this->corners.resize(4);
    this->sizes.resize(4);

    currentID = 0;

    // start timer
    this->time = 0;
    this->last_time = 0.0f;
    this->tick.start();
    this->timer.start();
}


void KilobotTracker::LOOPiterate()
{

    // wait for semaphores
    if ((srcUsed[0].available() > 0 && \
        srcUsed[1].available() > 0 && \
        srcUsed[2].available() > 0 && \
        srcUsed[3].available() > 0) || this->loadFirstIm)
    {

        // we have tracking, so it is safe to start the experiment
        if (!this->loadFirstIm && time == 0) {
            if (this->stage != IDENTIFY) {
                emit startExperiment(false /*we are not resuming the experiment*/);
            }
        }

        srcUsed[0].acquire();
        srcUsed[1].acquire();
        srcUsed[2].acquire();
        srcUsed[3].acquire();

        // process images into single image
        for (uint i = 0; i < 4; ++i) {
            this->warpedImages[i] = srcBuff[i][time % BUFF_SIZE].warped_image;
            this->warpedMasks[i] = srcBuff[i][time % BUFF_SIZE].warped_mask;
            this->corners[i] = srcBuff[i][time % BUFF_SIZE].corner;
            this->sizes[i] = srcBuff[i][time % BUFF_SIZE].size;
        }


        // feed with first frame
        if (time == 0) {
            compensator->feed(corners, warpedImages, warpedMasks);
        }

        // apply compensation
        for (int i = 0; i < 4; ++i) {
            compensator->apply(i, corners[i], srcBuff[i][time % BUFF_SIZE].full_warped_image, warpedMasks[i]);
        }

        Mat channels[4][3];

        // move full images from threads
        for (uint i = 0; i < 4; ++i) {

            Mat temp;
            srcBuff[i][time % BUFF_SIZE].full_warped_image.copyTo(temp);
            cv::split(temp, channels[i]);
            this->fullImages[i][0] = channels[i][0];
            this->fullImages[i][1] = channels[i][1];
            this->fullImages[i][2] = channels[i][2];
            /*cv::GaussianBlur(this->fullImages[i], this->fullImages[i], cv::Size(7,7), 3.5, 3.5);
            Mat temp;
            cv::GaussianBlur(this->fullImages[i], temp, cv::Size(17,17), 10.5, 10.5);
             this->fullImages[i] = this->fullImages[i] - temp;*/
            //srcBuff[i][time % BUFF_SIZE].full_warped_image.copyTo(this->fullImages[i]);
        }

#ifdef SLOW_WARP
        Mat result(2000,2000, CV_8UC1, Scalar(0,0,0));
        this->fullImages[clData.inds[0]][0](Rect(0,0,1000,1000)).copyTo(result(Rect(0,0,1000,1000)));
        this->fullImages[clData.inds[1]][0](Rect(1000,0,1000,1000)).copyTo(result(Rect(1000,0,1000,1000)));
        this->fullImages[clData.inds[2]][0](Rect(0,1000,1000,1000)).copyTo(result(Rect(0,1000,1000,1000)));
        this->fullImages[clData.inds[3]][0](Rect(1000,1000,1000,1000)).copyTo(result(Rect(1000,1000,1000,1000)));
#else
        Mat result;
        Mat top;
        hconcat(this->fullImages[clData.inds[0]][0](Rect(0,0,1000,1000)),this->fullImages[clData.inds[1]][0](Rect(0,0,1000,1000)),top);
        Mat bottom;
        hconcat(this->fullImages[clData.inds[2]][0](Rect(0,0,1000,1000)),this->fullImages[clData.inds[3]][0](Rect(0,0,1000,1000)),bottom);
        vconcat(top,bottom,result);
#endif

        srcFree[0].release();
        srcFree[1].release();
        srcFree[2].release();
        srcFree[3].release();


        this->finalImage = result;

        //qDebug() << "Main" << time << QTime::currentTime();

        switch (this->stage) {
        case TRACK:
            this->trackKilobots();
            break;
        case IDENTIFY:
            this->identifyKilobots();
            break;
        }

        ++time;

        if (time % 5 == 0) {
            float c_time = float(this->timer.elapsed())/1000.0f;
            emit errorMessage(QString("FPS = ") + QString::number(5.0f/(c_time-last_time)));
            last_time = c_time;
        }

    }

}

void KilobotTracker::updateKilobotStates()
{
    for (int i = 0; i < kilos.size(); ++i) {
        kilos[i]->updateExperiment();
        kilos[i]->updateHardware();
    }
}

void KilobotTracker::getInitialKilobotStates()
{
    for (int i = 0; i < kilos.size(); ++i) {
        kilos[i]->updateExperiment();
    }
}

void KilobotTracker::SETUPfindKilobots()
{

    if (this->finalImage.empty()) return;

    Mat res2;
    Mat display;
    this->finalImage.copyTo(display);

    res2 = this->finalImage;

    vector<Vec3f> circles;
    HoughCircles(res2,circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/ \
                 ,this->kbMaxSize-1/* circle distance*/ \
                 ,cannyThresh /* Canny threshold*/ \
                 ,houghAcc /*cicle algorithm accuracy*/ \
                 ,kbMinSize/* min circle size*/ \
                 ,kbMaxSize/* max circle size*/);

    qDebug() << "Found" << circles.size() << "circles";

    // the *2 is an assumption - should always be true...
    cv::cvtColor(display, display, CV_GRAY2RGB);

    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         //circle( result, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

    cv::resize(display,display,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));


    // convert to C header for easier mem ptr addressing
    IplImage imageIpl = display;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);

    setStitchedImage(pix);

    // generate kilobots
    this->kilos.clear();

    kilobot_colour col = OFF;

    for( size_t i = 0; i < circles.size(); i++ ) {

        this->kilos.push_back(new Kilobot(i,QPointF(circles[i][0],circles[i][1]),QPointF(1,1),col));

    }

    this->kiloHeadings.clear();
    this->kiloHeadings.resize(this->kilos.size());

}


void KilobotTracker::identifyKilobots()
{

    Mat display;
    cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);

    if (time == 0)
    {
        currentID = 0;
        identifyKilobot(currentID);
        qDebug() << "Try ID" << currentID;
    }

    if (time % 10 != 9) {
        for (uint i = 0; i < (uint) kilos.size(); ++i) {

            // get bounding box
            Rect bb = this->getKiloBotBoundingBox(i, 1.1f);

            Mat temp[3];

            // switch source depending on position...
            if (bb.x < 2000/2 && bb.y < 2000/2) {
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[0]][c](bb);
            } else if (bb.x > 2000/2-1 && bb.y < 2000/2) {
                Rect bb_adj = bb;
                bb_adj.x = bb_adj.x -1000;
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[1]][c](bb_adj);
            } else if (bb.x < 2000/2 && bb.y > 2000/2-1) {
                Rect bb_adj = bb;
                bb_adj.y = bb_adj.y -1000;
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[2]][c](bb_adj);
            } else if (bb.x > 2000/2-1 && bb.y > 2000/2-1) {
                Rect bb_adj = bb;
                bb_adj.x = bb_adj.x -1000;
                bb_adj.y = bb_adj.y -1000;
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[3]][c](bb_adj);
            }
            this->getKiloBotLightAdaptive(temp, Point(bb.width/2,bb.height/2),i);
        }
    }
    else
    {

        for (uint i = 0; i < (uint) kilos.size(); ++i) {

            // get bounding box
            Rect bb = this->getKiloBotBoundingBox(i, 1.1f);

            Mat temp[3];

            if (bb.x < 2000/2 && bb.y < 2000/2) {
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[0]][c](bb);
            } else if (bb.x > 2000/2-1 && bb.y < 2000/2) {
                Rect bb_adj = bb;
                bb_adj.x = bb_adj.x -1000;
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[1]][c](bb_adj);
            } else if (bb.x < 2000/2 && bb.y > 2000/2-1) {
                Rect bb_adj = bb;
                bb_adj.y = bb_adj.y -1000;
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[2]][c](bb_adj);
            } else if (bb.x > 2000/2-1 && bb.y > 2000/2-1) {
                Rect bb_adj = bb;
                bb_adj.x = bb_adj.x -1000;
                bb_adj.y = bb_adj.y -1000;
                for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[3]][c](bb_adj);
            }

            kiloLight light = this->getKiloBotLight(temp, Point(bb.width/2,bb.height/2),i);

            if (light.col == GREEN) {
                qDebug() << "Found ID" << currentID;
                kilos[i]->setID((uint8_t) currentID);
            }

        }
        // next id
        ++currentID;
        identifyKilobot(currentID);
        qDebug() << "Try ID" << currentID;
    }

    this->showMat(display);

}

void KilobotTracker::identifyKilobot(int id)
{

    // decompose id
    uint8_t data[9] = {0,0,0,0,0,0,0,0,0};
    data[0] = id >> 8;
    data[1] = id & 0xFF;

    kilobot_broadcast msg;
    msg.type = 120;
    msg.data = data;
    emit this->broadcastMessage(msg);

}

void KilobotTracker::trackKilobots()
{

    // convert for display
    Mat display;
    cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);

    switch (this->trackType) {
    {
    case CIRCLES_NAIVE:
        break;
    }
    {
    case CIRCLES_LOCAL:

            // setup the tracking region around each KB's last known position
            float maxDist = 31.0f*1.2f;//*this->kbMaxSize; // FIXED FOR NOW

            vector < Rect > bbs;

            // get the bounding box info for all KB
            for (uint i = 0; i < (uint) this->kilos.size(); ++i) {
                bbs.push_back(this->getKiloBotBoundingBox(i, 1.2f));
            }


            for (uint i = 0; i < (uint) this->kilos.size(); ++i) {


                Rect bb = bbs[i];
                Mat temp[3];

                // setup temp pars with previous KB state
                kiloLight light;
                light.col = kilos[i]->getLedColour();
                QPointF newPos = this->kilos[i]->getPosition();
                QPointF newVel = this->kilos[i]->getVelocity();

                // track light
                if (this->t_type & LED || this->t_type & ADAPTIVE_LED) {

                    // switch cam/vid source depending on position...
                    if (bb.x < 2000/2 && bb.y < 2000/2) {
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[0]][c](bb);
                    } else if (bb.x > 2000/2-1 && bb.y < 2000/2) {
                        Rect bb_adj = bb;
                        bb_adj.x = bb_adj.x -1000;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[1]][c](bb_adj);
                    } else if (bb.x < 2000/2 && bb.y > 2000/2-1) {
                        Rect bb_adj = bb;
                        bb_adj.y = bb_adj.y -1000;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[2]][c](bb_adj);
                    } else if (bb.x > 2000/2-1 && bb.y > 2000/2-1) {
                        Rect bb_adj = bb;
                        bb_adj.x = bb_adj.x -1000;
                        bb_adj.y = bb_adj.y -1000;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[3]][c](bb_adj);
                    }

                    if (this->t_type & ADAPTIVE_LED) {
                        light = this->getKiloBotLightAdaptive(temp, Point(bb.width/2,bb.height/2),i);
                    } else if (this->t_type & LED){
                        light = this->getKiloBotLight(temp, Point(bb.width/2,bb.height/2),i);
                    }

                    // TEMP
                    /*if (i == 0) {
                        if (t_type & ADAPTIVE_LED) {
                            Mat t;
                            cv::resize(this->testAdap,t,Size(),4,4,INTER_NEAREST);
                            cv::cvtColor(t, t, CV_GRAY2RGB);
                            t = t*20.0f;
                            t.copyTo(display(Rect(0,0,t.size[1],t.size[0])));

                        }
                    }*/
                }


                if (this->t_type & POS) {

                    bool no_match = true;
                    int circle_acc = 30;

                    vector<Vec3f> circles;

                    while (no_match && circle_acc > 10) {

                        // try fixed pars
                        int minS = this->kbMinSize;//9;
                        int maxS = this->kbMaxSize;//25;

                        HoughCircles(temp[0],circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/,1.0/*maxS-1.0 circle distance*/,this->cannyThresh /* Canny threshold*/,circle_acc /*cicle algorithm accuracy*/,minS/* min circle size*/,maxS/* max circle size*/);

                        if (circles.size() > 0) no_match = false;

                        circle_acc -= 2;

                    }


                    if (circles.size() > 0) {
                        // take the nearest one

                        float reduceMaxSpeed = 10.0;

                        int best_index = 0;
                        for (uint k = 0; k < circles.size(); ++k) {

                            bool oops = false;

                            // full exclusion
                            for (uint l = 0; l < (uint) this->kilos.size(); ++l) {

                                if (l == i) continue;

                                Point cCent(circles[k][0]-bb.width/2+kilos[i]->getPosition().x(), circles[k][1]-bb.height/2+kilos[i]->getPosition().y());

                                if ( qPow(cCent.x - kilos[l]->getPosition().x(),2) + qPow(cCent.y - kilos[l]->getPosition().y(),2) \
                                       <  qPow(circles[k][0]-bb.width/2,2) + qPow(circles[k][1]-bb.height/2,2) ) {
                                    circles.erase(circles.begin()+k);
                                    --k;
                                    oops = true;
                                    break;
                                }

                            }

                            if (oops) continue;

                            best_index = (this->kbMaxSize-reduceMaxSpeed)*(this->kbMaxSize-reduceMaxSpeed) \
                                    > (circles[k][0]-bb.width/2)*(circles[k][0]-bb.width/2)+(circles[k][1]-bb.height/2)*(circles[k][1]-bb.height/2) \
                                    && (circles[k][0]-bb.width/2)*(circles[k][0]-bb.width/2)+(circles[k][1]-bb.height/2)*(circles[k][1]-bb.height/2) \
                                    < (circles[best_index][0]-bb.width/2)*(circles[best_index][0]-bb.width/2)+(circles[best_index][1]-bb.height/2)*(circles[best_index][1]-bb.height/2) ? i : best_index;

                        }

                        if (circles.size() != 0) {

                            // if we haven't moved too far
                            if ((this->kbMaxSize-reduceMaxSpeed)*(this->kbMaxSize-reduceMaxSpeed) > (circles[best_index][0]-bb.width/2)*(circles[best_index][0]-bb.width/2)+(circles[best_index][1]-bb.height/2)*(circles[best_index][1]-bb.height/2))
                            {

                                float smooth_fact = 0.5;

                                int new_x = float(bb.x+circles[best_index][0])*(1.0-smooth_fact) + float(kilos[i]->getPosition().x())*smooth_fact;
                                int new_y = float(bb.y+circles[best_index][1])*(1.0-smooth_fact) + float(kilos[i]->getPosition().y())*smooth_fact;

                                // update velocity
                                QPointF prevPos = this->kilos[i]->getPosition();
                                QPointF prevVel = this->kilos[i]->getVelocity();

                                newPos = QPointF(new_x,new_y);
                                newVel = prevVel*(2.0f/3.0f) + (newPos - prevPos)*(1.0f/3.0f);

                            }

                        }

                    }
                } // END POS

                if (this->t_type & ROT && (this->t_type & LED || this->t_type & ADAPTIVE_LED)) {

                    QLineF lightLine = QLineF(QPointF(0,0),QPointF(light.pos.x,light.pos.y));
                    QLineF velLine = QLineF(QPointF(0,0),newVel);

                    // if we have a light
                    if (light.col != OFF) {

                        if (velLine.length() < 1.0f) {
                            lightLine.setLength(0.9f);
                            lightLine.setAngle(lightLine.angle() + 20.0f);
                            newVel = lightLine.p2();
                        } else {
                            // combine LED and velocity estimates
                            lightLine.setLength(velLine.length());
                            // align to forward
                            lightLine.setAngle(lightLine.angle() + 20.0f);
                            newVel = (lightLine.p2() + velLine.p2())*0.5f;
                        }

                    }
                }

                // put in any new data
                this->kilos[i]->updateState(newPos,newVel,light.col);

                // DRAW
                Point center(round(kilos[i]->getPosition().x()), round(kilos[i]->getPosition().y()));
                if (kilos[i]->getID() == UNASSIGNED_ID) {
                     // not id'd
                    circle( display, center, 1, Scalar(255,0,0), 3, 8, 0 );
                } else {
                     // id'd
                    circle( display, center, 1, Scalar(0,255,0), 3, 8, 0 );
                }
                // plot
                QLineF currVel = QLineF(QPointF(0,0),this->kilos[i]->getVelocity());
                currVel.setLength(currVel.length()*10.0f+20.0f);
                QPointF hdQpt = currVel.p2() + this->kilos[i]->getPosition();
                Point heading(hdQpt.x(), hdQpt.y());
                switch (light.col) {
                case RED:
                     line(display,center,heading,Scalar(255,0,0),3);
                     break;
                case GREEN:
                     line(display,center,heading,Scalar(0,255,0),3);
                     break;
                case BLUE:
                     line(display,center,heading,Scalar(0,0,255),3);
                     break;
                case OFF:
                     line(display,center,heading,Scalar(255,255,255),3);
                     break;
                }
            }

            if (kilos.size() > 0) {
                Rect bb;
                bb.x = cvRound(this->kilos[0]->getPosition().x() - maxDist);
                bb.y = cvRound(this->kilos[0]->getPosition().y() - maxDist);
                bb.width = cvRound(maxDist*2.0f);
                bb.height = cvRound(maxDist*2.0f);
                rectangle(display, bb, Scalar(0,0,255),3);
            }

            break;
    }
    {
    case MY_HAPPY_OTHER_TRACKER:
        // GIOVANNI IS WRITING THIS
        break;
    }
    }

    this->drawOverlay(display);
    this->showMat(display);

}

void KilobotTracker::drawOverlay(Mat & display)
{

    for (int i = 0; i < this->circsToDraw.size(); ++i) {

        cv::circle(display,this->circsToDraw[i].pos,this->circsToDraw[i].r,Scalar(this->circsToDraw[i].col.red(),this->circsToDraw[i].col.green(),this->circsToDraw[i].col.blue()),2);

    }

}

Rect KilobotTracker::getKiloBotBoundingBox(int i, float scale)
{


    float maxDist = scale*this->kbMaxSize;

    Rect bb;
    bb.x = cvRound(this->kilos[i]->getPosition().x() - maxDist);
    bb.y = cvRound(this->kilos[i]->getPosition().y() - maxDist);
    bb.width = cvRound(maxDist*2.0f);
    bb.height = cvRound(maxDist*2.0f);

    bb.x = bb.x > 0 ? bb.x : 0;
    bb.width = bb.x + bb.width < this->finalImage.size().width ? bb.width :  this->finalImage.size().width - bb.x - 1;
    bb.y = bb.y > 0 ? bb.y : 0;
    bb.height = bb.y + bb.height < this->finalImage.size().height ? bb.height :  this->finalImage.size().height - bb.y - 1;

    return bb;

}


kiloLight KilobotTracker::getKiloBotLight(Mat channels[3], Point centreOfBox, int index)
{
    // find the location and colour of the light...

    kiloLight light;
    light.pos = Point(-1,-1);

    Mat temp[3];
    Scalar sums[3];

    float tooBig = 10000.0f;

    uint maxIndex = 0;

    // find colour
    for (uint i = 0; i < 3; ++i) {
        cv::threshold(channels[i], temp[i], kilos[index]->lightThreshold,255,CV_THRESH_TOZERO);
        temp[i] = temp[i] - kilos[index]->lightThreshold;
        sums[i] = cv::sum(temp[i]);
        maxIndex = sums[i][0] > sums[maxIndex][0] ? i : maxIndex;
    }

    // set the light colour (OFF = 0, RED = 1, GREEN = 2, BLUE = 3)
    light.col = sums[maxIndex][0] > 0.0f && sums[maxIndex][0] < tooBig ? (lightColour) (maxIndex+1) : OFF;

    cv::Moments m = moments(temp[maxIndex], true);
    cv::Point centreOfLight(m.m10/m.m00, m.m01/m.m00);

    // calculate the heading:
    if (centreOfLight.x > -1 && centreOfLight.y > -1) {

        light.pos = centreOfLight - centreOfBox;

    }

    return light;

}

kiloLight KilobotTracker::getKiloBotLightAdaptive(Mat channels[3], Point centreOfBox, int index)
{
    // find the location and colour of the light...

    kiloLight light;
    light.pos = Point(-1,-1);

    vector < Mat > temp(3);
    Scalar sums[3];

    float tooBig = 2000.0f;//10000.0f
    int step = 5;

    uint maxIndex = 0;

    // find colour
    for (uint i = 0; i < 3; ++i) {
        cv::threshold(channels[i], temp[i], kilos[index]->lightThreshold,255,CV_THRESH_TOZERO);
        temp[i] = temp[i] - kilos[index]->lightThreshold;
        sums[i] = cv::sum(temp[i]);
        maxIndex = sums[i][0] > sums[maxIndex][0] ? i : maxIndex;
    }

    if (sums[maxIndex][0] > tooBig) {
        kilos[index]->lightThreshold = kilos[index]->lightThreshold + step < 255 ? kilos[index]->lightThreshold + step : kilos[index]->lightThreshold;
    }
    if (sums[maxIndex][0] < 1.0f) {
        kilos[index]->lightThreshold = kilos[index]->lightThreshold - step > 100 ? kilos[index]->lightThreshold - step : kilos[index]->lightThreshold;

        maxIndex = 0;

        // move back up if we hit a bit prob
        for (uint i = 0; i < 3; ++i) {
            cv::threshold(channels[i], temp[i], kilos[index]->lightThreshold,255,CV_THRESH_TOZERO);
            temp[i] = temp[i] - kilos[index]->lightThreshold;
            sums[i] = cv::sum(temp[i]);
            maxIndex = sums[i][0] > sums[maxIndex][0] ? i : maxIndex;
        }

        if (sums[maxIndex][0] > tooBig) {
            kilos[index]->lightThreshold = kilos[index]->lightThreshold + step < 255 ? kilos[index]->lightThreshold + step : kilos[index]->lightThreshold;
        }
    }

    // set the light colour (OFF = 0, RED = 1, GREEN = 2, BLUE = 3)
    light.col = sums[maxIndex][0] > 0.0f && sums[maxIndex][0] < tooBig ? (lightColour) (maxIndex+1) : OFF;

    cv::Moments m = moments(temp[maxIndex], true);
    cv::Point centreOfLight(m.m10/m.m00, m.m01/m.m00);

    // calculate the heading:
    if (centreOfLight.x > -1 && centreOfLight.y > -1) {

        light.pos = centreOfLight - centreOfBox;

    }



    /*if (index == 0) {

        cv::hconcat(temp, this->testAdap);
        qDebug() << kilos[index]->lightThreshold;

        if (light.col == RED) qDebug() << "RED";
        if (light.col == BLUE) qDebug() << "BLUE";
        if (light.col == GREEN) qDebug() << "GREEN";
        if (light.col == OFF) qDebug() << "OFF";
    }*/

    return light;

}

void KilobotTracker::SETUPloadCalibration()
{

    // Load the calibration data

    // nicety - load last used directory
    QSettings settings;
    QString lastDir = settings.value("lastDirOut", QDir::homePath()).toString();
    QString fileName = QFileDialog::getOpenFileName((QWidget *) sender(), tr("Load Calibration"), lastDir, tr("XML files (*.xml);; All files (*)"));

    if (fileName.isEmpty()) {
        emit errorMessage("No file selected");
        return;
    }

    // load the data
    FileStorage fs(fileName.toStdString(),FileStorage::READ);

    fs["corner1"] >> this->arenaCorners[0];
    fs["corner2"] >> this->arenaCorners[1];
    fs["corner3"] >> this->arenaCorners[2];
    fs["corner4"] >> this->arenaCorners[3];

    fs["R"] >> this->Rs;
    fs["K"] >> this->Ks;

    // need more sanity checks than this...
    if (Rs.size() != 4 || Ks.size() != 4) {
        emit errorMessage("Invalid calibration data");
        this->haveCalibration = false;
        return;
    }

    for (uint i = 0; i < Rs.size(); ++i) {
        Mat R;
        Rs[i].convertTo(R, CV_32F);
        Rs[i] = R;
    }
    for (uint i = 0; i < Ks.size(); ++i) {
        Mat K;
        Ks[i].convertTo(K, CV_32F);
        Ks[i] = K;
    }

    QDir lastDirectory (fileName);
    lastDirectory.cdUp();
    settings.setValue ("lastDirOut", lastDirectory.absolutePath());

    this->SETUPstitcher();

    // load first images

    // launch threads
    this->THREADSlaunch();

    if (!this->compensator) {
        // calculate to compensate for exposure
        compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);
    }

    if (!this->blender) {
        // blend the images
        blender = detail::Blender::createDefault(detail::Blender::FEATHER, true);
    }

    this->warpedImages.resize(4);
    this->warpedMasks.resize(4);
    this->corners.resize(4);
    this->sizes.resize(4);

    this->time = 0;

    // run stitcher once
    this->loadFirstIm = true;
    this->LOOPiterate();
    this->loadFirstIm = false;

    this->THREADSstop();

    this->time = 0;

    this->haveCalibration = true;

}

void KilobotTracker::SETUPstitcher()
{

    // initial config
    Ptr<WarperCreator> warper_creator;
    warper_creator = new cv::PlaneWarper();//makePtr<cv::PlaneWarper>();
    Ptr<detail::RotationWarper> warper = warper_creator->create(2000.0f);

    Mat in(IM_HEIGHT, IM_WIDTH, CV_8UC3, Scalar(0,0,0));
    Mat out;

    corners.resize(4);
    sizes.resize(4);

    for (uint i = 0; i < 4; ++i) {
        this->corners[i] = warper->warp(in, Ks[i], Rs[i], INTER_LINEAR, BORDER_REFLECT, out);
        this->sizes[i] = out.size();
    }


   int min_x = INT_MAX;
   int min_y = INT_MAX;
   int max_x = -INT_MAX;
   int max_y = -INT_MAX;

   for (int j = 0; j < 4; ++j) {
       if (corners[j].x < min_x) min_x = corners[j].x;
       if (corners[j].y < min_y) min_y = corners[j].y;
       if (corners[j].x + sizes[j].width > max_x) max_x = corners[j].x + sizes[j].width;
       if (corners[j].y + sizes[j].height > max_y) max_y = corners[j].y + sizes[j].height;
   }

   fullSize =  Size(max_x-min_x+1, max_y-min_y+1);
   fullCorner =  Point(min_x, min_y);

   // assign indices...
   for (int j = 0; j < 4; ++j) {
       if (corners[j].x - fullCorner.x < fullSize.width/4 && corners[j].y - fullCorner.y < fullSize.height/4) {
        clData.inds[0] = j;
       } else if (corners[j].x - fullCorner.x > fullSize.width/4 && corners[j].y - fullCorner.y < fullSize.height/4) {
        clData.inds[1] = j;
       } else if (corners[j].x - fullCorner.x < fullSize.width/4 && corners[j].y - fullCorner.y > fullSize.height/4) {
        clData.inds[2] = j;
       } else if (corners[j].x - fullCorner.x > fullSize.width/4 && corners[j].y - fullCorner.y > fullSize.height/4) {
        clData.inds[3] = j;
       }
   }

}

void KilobotTracker::THREADSlaunch()
{
    for (uint i = 0; i < 4; ++i)
    {
        if (srcStop[i].available()) srcStop[i].acquire();
        if (!this->threads[i]) {
            this->threads[i] = new acquireThread;
        }
        this->threads[i]->arenaCorners[0] = this->arenaCorners[0];
        this->threads[i]->arenaCorners[1] = this->arenaCorners[1];
        this->threads[i]->arenaCorners[2] = this->arenaCorners[2];
        this->threads[i]->arenaCorners[3] = this->arenaCorners[3];
        this->threads[i]->corner = this->corners[i];
        this->threads[i]->size = this->sizes[i];
        this->threads[i]->fullCorner = fullCorner;
        this->threads[i]->fullSize = fullSize;
        this->threads[i]->R = this->Rs[i];
        this->threads[i]->K = this->Ks[i];
        this->threads[i]->keepRunning = true;
        this->threads[i]->index = i;
        this->threads[i]->type = this->srcType;
        this->threads[i]->videoDir = this->videoPath;
        this->threads[i]->start();
    }
}

void KilobotTracker::THREADSstop()
{

    // stop the timer
    if (tick.isActive()) {
        this->tick.stop();
    }

    // close threads
    srcStop[0].release();
    srcStop[1].release();
    srcStop[2].release();
    srcStop[3].release();

    // reset semaphores
    for (uint i = 0; i < 4; i++) {
        this->threads[i]->wait();
        while (srcFree[i].available()) {
            srcFree[i].acquire();
        }
        while (srcUsed[i].available()) {
            srcUsed[i].acquire();
        }
        srcFree[i].release(BUFF_SIZE);
    }


}

void KilobotTracker::showMat(Mat &display)
{
    // display
    cv::resize(display,display,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));

    // convert to C header for easier mem ptr addressing
    IplImage imageIpl = display;

    // create a QImage container pointing to the image data
    QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

    // assign to a QPixmap (may copy)
    QPixmap pix = QPixmap::fromImage(qimg);

    emit setStitchedImage(pix);
}

void KilobotTracker::SETUPsetCamOrder()
{

    int temp[4];
    // check we have numbers zero to three
    bool haveIndex[4] = {false,false,false,false};

    QLineEdit * src = qobject_cast < QLineEdit * > (this->sender());
    if (src) {
        QString str = src->text();
        QStringList list = str.split(",");
        if (list.size() == 4)
        {
            for (int i = 0; i < list.size(); ++i) {
                temp[i] = list[i].toInt();
                if (temp[i] < 4 && temp[i] > -1) {
                    haveIndex[temp[i]] = true;
                }
            }
        }
        if (haveIndex[0] && haveIndex[1] && haveIndex[2] && haveIndex[3])
        {
            for (int i = 0; i < 4; ++i) camOrder[i] = temp[i];
        }
    }

}



