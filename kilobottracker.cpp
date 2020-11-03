/*!
 * Kilobottracker.cpp
 *
 *  Created on: 3 Oct 2016
 *  Author: Alex Cope
 */

#include "kilobottracker.h"
#include "kilobotexperiment.h"
#include <QImage>
#include <QThread>
#include <QLineEdit>
#include <QDir>
#include <QSettings>
#include <QFileDialog>
#include <QtMath>
#include <QDebug>

//#define TEST_WITHOUT_CAMERAS
//#define TESTLEDS
#define COLDET_V1

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
    int height_x_adj;
    int height_y_adj;

    // the index of the source
    uint index = 0;

    bool keepRunning = true;


    srcDataType type = CAMERA;

    cv::VideoCapture cap;

    //video saving
    bool savecamerasframes=false;
    QString videoframeprefix;

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

#ifdef USE_CUDA
        // if using CUDA we need a stream to make the operations thread safe
        cuda::Stream stream;
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
                    // NOTE: need to decide on format for imagevideos
                    image = imread((this->videoDir+QDir::toNativeSeparators("/")+QString("frame_00200_")+QString::number(index)+QString(".jpg")).toStdString());
                }
                else if (type == CAMERA) {
#ifndef TEST_WITHOUT_CAMERAS

                    // Open the camera
                    camUsage.acquire();
                    if (!cap.isOpened() && camOrder[index]<4) {
                        cap.open(camOrder[index]);
                        // set REZ
                        if (cap.isOpened()) {
                            cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));
                            cap.set(CV_CAP_PROP_FRAME_WIDTH, IM_WIDTH);
                            cap.set(CV_CAP_PROP_FRAME_HEIGHT, IM_HEIGHT);
                        } else {
                            this->keepRunning = false;
                            continue;
                        }
                    }

                    if (cap.isOpened()) {
                        // exhaust buffer
                        cap.grab();
                        cap.grab();
                        camUsage.release();

                        camUsage.acquire();
                        cap.retrieve(image);
                        if(savecamerasframes) {
                            vector<int> compression_params;
                            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                            compression_params.push_back(95);
                            imwrite(videoframeprefix.toStdString(), image, compression_params);
                            savecamerasframes=false;
                        }
                        camUsage.release();
                    }
                    else
#endif
                    {
                        image = Mat(IM_HEIGHT,IM_WIDTH, CV_8UC3, Scalar(0,0,0)); /* TEMPORARY!!! */
                        camUsage.release();
                    }

                } else if (type == VIDEO) {
                    qDebug() << (this->videoDir+QDir::toNativeSeparators("/")+QString("frame_%1_%2").arg(/*time+*/time, 5,10, QChar('0')).arg(index)+QString(".jpg"));
                    image = imread((this->videoDir+QDir::toNativeSeparators("/")+QString("frame_%1_%2").arg(/*time+*/time, 5,10, QChar('0')).arg(index)+QString(".jpg")).toStdString());
                    if (image.empty()) {qDebug() << "Image not found"; continue;}
                }

                // Prepare images masks
                if (mask.size().width < 10) {
                    mask.create(image.size(), CV_8U);
                }
                mask.setTo(Scalar::all(255));

                // check semaphore
                srcFree[index].acquire();

#ifdef USE_CUDA
                Mat tempCuda;
                srcBuff[index][time % BUFF_SIZE].corner = warper->warp(image, K, R, INTER_LINEAR, BORDER_REFLECT, tempCuda);
                srcBuff[index][time % BUFF_SIZE].warped_image.upload(tempCuda);
                srcBuff[index][time % BUFF_SIZE].size = srcBuff[index][time % BUFF_SIZE].warped_image.size();
                warper->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, tempCuda);
                srcBuff[index][time % BUFF_SIZE].warped_mask.upload(tempCuda);
#else
                srcBuff[index][time % BUFF_SIZE].corner = warper->warp(image, K, R, INTER_LINEAR, BORDER_REFLECT, srcBuff[index][time % BUFF_SIZE].warped_image);
                srcBuff[index][time % BUFF_SIZE].size = srcBuff[index][time % BUFF_SIZE].warped_image.size();
                warper->warp(mask, K, R, INTER_NEAREST, BORDER_CONSTANT, srcBuff[index][time % BUFF_SIZE].warped_mask);
#endif

                // only do this if we are not loading calibration
                if (!(this->corner.x == -1 && this->corner.y == -1)) {

                    // adjustment used to compensate for placing calibration images on table, and not at kilobot height
                    MAT_TYPE temp;
#ifdef USE_CUDA
//                    qDebug() << "Size w: " << size.width << " h: " << size.height;
//                    qDebug() << "Full-Size w: " << fullSize.width << " h: " << fullSize.height;
//                    cv::cuda::resize(srcBuff[index][time % BUFF_SIZE].warped_image, temp, Size(size.width-height_adj,size.height-height_adj),0,0, INTER_LINEAR, stream);
//                    cv::cuda::resize(temp, temp2,Size((2048*(size.width-height_adj))/fullSize.width,(1536*(size.height-height_adj))/fullSize.height),0,0, INTER_LINEAR, stream);
                    cv::cuda::resize(srcBuff[index][time % BUFF_SIZE].warped_image, temp, Size((min(IM_HEIGHT,IM_WIDTH)*(size.width-height_x_adj))/fullSize.width,(min(IM_HEIGHT,IM_WIDTH)*(size.height-height_y_adj))/fullSize.height),0,0, INTER_LINEAR, stream);
#else
                    cv::resize(srcBuff[index][time % BUFF_SIZE].warped_image, temp,Size((min(IM_HEIGHT,IM_WIDTH)*(size.width-height_x_adj))/fullSize.width,(min(IM_HEIGHT,IM_WIDTH)*(size.height-height_y_adj))/fullSize.height));
#endif

                    Point2f arenaCorners_adj[4];
                    Point2f outputQuad_adj[4];
//                    qDebug() << "Thread " << this->index << " corner " << corner.x << "," << corner.y << " full Corner " << fullCorner.x << "," << fullCorner.y;
                    for (int i = 0; i < 4; ++i) {
                        arenaCorners_adj[i] = arenaCorners[i] - Point2f(((corner.x-fullCorner.x)*min(IM_HEIGHT,IM_WIDTH))/fullSize.width,((corner.y-fullCorner.y)*min(IM_HEIGHT,IM_WIDTH))/fullSize.height);
//                        qDebug() << "arenaCorners_adj " << i << " is " << arenaCorners_adj[i].x << "," << arenaCorners_adj[i].y;
                        // shift the location for all but the first camera, and add 100 pixel overlap around the images
                        outputQuad_adj[i] = outputQuad[i] - Point2f((fabs(corner.x-fullCorner.x)>100)*1000-100, (fabs(corner.y-fullCorner.y)>100)*1000-100);
                    }

                    Mat M = getPerspectiveTransform(arenaCorners_adj,outputQuad_adj);
                    // 1200 x 1200 output includes 100 pixel overlap aound entire image
#ifdef USE_CUDA
                    cuda::warpPerspective(temp, srcBuff[index][time % BUFF_SIZE].full_warped_image, M, Size(1200,1200),INTER_LINEAR,BORDER_CONSTANT,Scalar(),stream);
#else
                    warpPerspective(temp, srcBuff[index][time % BUFF_SIZE].full_warped_image, M, Size(1200,1200));
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
    // select cuda device
#ifdef USE_CUDA
    //qDebug() << "There are" << cuda::getCudaEnabledDeviceCount() << "CUDA devices";
    try {
        cuda::setDevice(cuda::getCudaEnabledDeviceCount()-2);
    } catch (cv::Exception){
        cuda::setDevice(cuda::getCudaEnabledDeviceCount()-1);
    }
    qDebug() << "Using CUDA device " << cuda::DeviceInfo().name();

#endif

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
void KilobotTracker::LOOPstartstop(int expType)
{

    this->expType = (experimentType) expType;
    // update the run/stop button
    emit toggleExpButton((int)expType);

    // check if running
    if (this->threads[0] && this->threads[0]->isRunning()) {
        // reset IDing
        //this->aStage = START;
        currentID = 0;

        emit errorMessage(QString("FPS = ") + QString::number(float(time)/(float(timer.elapsed())/1000.0f)));
        this->THREADSstop();
        // Stop the experiment
        if (this->expType != IDENTIFY) {
            emit stopExperiment();
        }

        QThread::currentThread()->setPriority(QThread::NormalPriority);

        // Reset the time to zero for the next experiment
        time=0;
        return;

    }

    emit clearMsgQueue();

#ifdef USE_CUDA
    // setup kb initial locs
    Mat tempKbLocs(1,kilos.size(), CV_32FC2);
    float * data = (float *) tempKbLocs.data;
    for (int i = 0; i < kilos.size(); ++i) {
        data[i*2] = kilos[i]->getPosition().x();
        data[i*2+1] = kilos[i]->getPosition().y();
    }
    kbLocs.upload(tempKbLocs);
    
    //this->hough = cuda::createHoughCirclesDetector(1.0,1.0,this->cannyThresh,this->houghAcc,this->kbMinSize,this->kbMaxSize,5000); // kilobot detection
    //this->hough = cuda::createHoughCirclesDetector(1.0,this->kbMinSize/1.4,this->cannyThresh,this->houghAcc,this->kbMinSize,this->kbMaxSize,3000); // kilobot detection
    this->hough = cuda::createHoughCirclesDetector(1.0,1.0,this->cannyThresh,this->houghAcc,this->kbMinSize,this->kbMaxSize,20000); // kilobot detection
    this->hough2 = cuda::createHoughCirclesDetector(1.0,1.0,this->cannyThresh,this->houghAcc,this->kbMinSize/7,this->kbMinSize*10/7,10000);// led detection
    //this->hough2 = cuda::createHoughCirclesDetector(1.0,1.0,this->cannyThresh,this->houghAcc,1,this->kbMinSize*10/7,10000);// led detection

    clahe=cuda::createCLAHE(10);
    dilateFilter=cuda::createMorphologyFilter(MORPH_DILATE,CV_8UC1, element);

#endif

    QThread::currentThread()->setPriority(QThread::TimeCriticalPriority);

    // only if we have calib data
    if (!this->haveCalibration) {
        return;
    }

    // when the ID-ASSIGNMENT starts, new sequential (random) IDs are assigned to each ARK-Kilobot at the start of the process (to avoid duplicates)
    if (this->expType == ID_ASSIGNMENT){
        for (int i = 0; i < this->kilos.size(); ++i) {
            this->kilos[i]->setID(i);
        }
    }

    // launch threads
    this->THREADSlaunch();

    // connect kilobots
    for (int i = 0; i < this->kilos.size(); ++i) {
        this->kilos[i]->disconnect(SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)));
        connect(this->kilos[i],SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)), this->expt, SLOT(setupInitialStateRequiredCode(Kilobot*,Kilobot)));
    }

    /** @Salah: check if the compensator and blender is ever used */
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

    // reset tracking vectors
    this->lost_count.clear();
    this->lost_count.resize(this->kilos.size());
    this->pendingRuntimeIdentification.clear();
    this->m_ongoingRuntimeIdentification = false;
    this->m_runtimeIdentificationTimer = 0;

}
void KilobotTracker::LOOPiterate()
{

    if(savecamerasframes && (time <= numberofframes)){

        if(!(this->threads[0]->savecamerasframes) &&
           !(this->threads[1]->savecamerasframes) &&
           !(this->threads[2]->savecamerasframes) &&
           !(this->threads[3]->savecamerasframes)){
            for (uint i = 0; i < 4; ++i)
            {
                this->threads[i]->savecamerasframes = true;
                this->threads[i]->videoframeprefix=savecamerasframesdir+QString("/")+QString("frame_%1_%2").arg(/*time+*/time-1, 5,10, QChar('0')).arg(i)+QString(".jpg");
            }
        }
        else return;
    }
    else savecamerasframes=false;
    // wait for semaphores
    if ((srcUsed[0].available() > 0 && \
         srcUsed[1].available() > 0 && \
         srcUsed[2].available() > 0 && \
         srcUsed[3].available() > 0) || this->loadFirstIm)
    {

        // we have tracking, so it is safe to start the experiment
        if (!this->loadFirstIm && time == 0) {
            if (this->expType != IDENTIFY) {
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
#ifndef USE_CUDA
        if (time == 0) {
            compensator->feed(corners, warpedImages, warpedMasks);
        }

        // apply compensation
        for (int i = 0; i < 4; ++i) {
            compensator->apply(i, corners[i], srcBuff[i][time % BUFF_SIZE].full_warped_image, warpedMasks[i]);
        }
#endif

#ifdef USE_CUDA
        cuda::GpuMat channels[4][3];
#else
        Mat channels[4][3];
#endif

        Mat saveIm[4];

        // move full images from threads
        for (uint i = 0; i < 4; ++i) {

#ifdef USE_CUDA
            cuda::GpuMat temp;
#else
            Mat temp;
#endif
            srcBuff[i][time % BUFF_SIZE].full_warped_image.copyTo(temp);
            CV_NS split(temp, channels[i]);
#ifdef USE_CUDA
            temp.download(saveIm[i]);
#else
            saveIm[i] = temp;
#endif
            this->fullImages[i][0] = channels[i][0];
            this->fullImages[i][1] = channels[i][1];
            this->fullImages[i][2] = channels[i][2];

        }

        Mat top;
        Mat bottom;
#ifdef USE_CUDA
        cv::cuda::GpuMat resultB(2000, 2000, this->fullImages[clData.inds[0]][0].type());
        this->fullImages[clData.inds[0]][0](Rect(100,100,1000,1000)).copyTo(resultB(cv::Rect(0,0,1000,1000)));
        this->fullImages[clData.inds[1]][0](Rect(100,100,1000,1000)).copyTo(resultB(cv::Rect(1000,0,1000,1000)));
        this->fullImages[clData.inds[2]][0](Rect(100,100,1000,1000)).copyTo(resultB(cv::Rect(0,1000,1000,1000)));
        this->fullImages[clData.inds[3]][0](Rect(100,100,1000,1000)).copyTo(resultB(cv::Rect(1000,1000,1000,1000)));
        cv::cuda::GpuMat resultG(2000, 2000, this->fullImages[clData.inds[0]][0].type());
        this->fullImages[clData.inds[0]][1](Rect(100,100,1000,1000)).copyTo(resultG(cv::Rect(0,0,1000,1000)));
        this->fullImages[clData.inds[1]][1](Rect(100,100,1000,1000)).copyTo(resultG(cv::Rect(1000,0,1000,1000)));
        this->fullImages[clData.inds[2]][1](Rect(100,100,1000,1000)).copyTo(resultG(cv::Rect(0,1000,1000,1000)));
        this->fullImages[clData.inds[3]][1](Rect(100,100,1000,1000)).copyTo(resultG(cv::Rect(1000,1000,1000,1000)));
        cv::cuda::GpuMat resultR(2000, 2000, this->fullImages[clData.inds[0]][0].type());
        this->fullImages[clData.inds[0]][2](Rect(100,100,1000,1000)).copyTo(resultR(cv::Rect(0,0,1000,1000)));
        this->fullImages[clData.inds[1]][2](Rect(100,100,1000,1000)).copyTo(resultR(cv::Rect(1000,0,1000,1000)));
        this->fullImages[clData.inds[2]][2](Rect(100,100,1000,1000)).copyTo(resultR(cv::Rect(0,1000,1000,1000)));
        this->fullImages[clData.inds[3]][2](Rect(100,100,1000,1000)).copyTo(resultR(cv::Rect(1000,1000,1000,1000)));
#else
        Mat result;
        hconcat(this->fullImages[clData.inds[0]][0](Rect(100,100,1000,1000)),this->fullImages[clData.inds[1]][0](Rect(100,100,1000,1000)),top);
        hconcat(this->fullImages[clData.inds[2]][0](Rect(100,100,1000,1000)),this->fullImages[clData.inds[3]][0](Rect(100,100,1000,1000)),bottom);
        vconcat(top,bottom,result);
#endif

        hconcat(saveIm[clData.inds[0]](Rect(100,100,1000,1000)),saveIm[clData.inds[1]](Rect(100,100,1000,1000)),top);
        hconcat(saveIm[clData.inds[2]](Rect(100,100,1000,1000)),saveIm[clData.inds[3]](Rect(100,100,1000,1000)),bottom);
        vconcat(top,bottom,this->finalImageCol);


        srcFree[0].release();
        srcFree[1].release();
        srcFree[2].release();
        srcFree[3].release();

        if (flipangle!=0) {
            cv::Point origin(this->finalImageB.cols/2,this->finalImageB.rows/2);
            Mat rotationmatrixCPU;
            rotationmatrixCPU=getRotationMatrix2D(origin,flipangle,1);

            cuda::warpAffine(resultB,this->finalImageB,rotationmatrixCPU,resultB.size());
            cuda::warpAffine(resultG,this->finalImageG,rotationmatrixCPU,resultG.size());
            cuda::warpAffine(resultR,this->finalImageR,rotationmatrixCPU,resultR.size());
        } else {
            this->finalImageB = resultB;
            this->finalImageG = resultG;
            this->finalImageR = resultR;
        }

        switch (this->expType) {
        case USER_EXP:{
            this->trackKilobots();

            /** determine if executing the runtime-identification (RTI) */
            if (this->m_runtimeIDenabled && !experimentIsBroadcasting)
            {
                bool runRTI = false;
                if (this->m_ongoingRuntimeIdentification){ // if already running, keep running
                    runRTI = true;
                } else if (!this->pendingRuntimeIdentification.empty() && this->m_runtimeIdentificationTimer++ > 50){ // if there are pending request for more than 5s
                    // I check that all pending request are not lost robots
                    bool anyLost = false;
                    for (int i=0; i<this->pendingRuntimeIdentification.size(); ++i){
                        if (this->lost_count[this->pendingRuntimeIdentification[i]] > 0){
                            anyLost = true;
                            break;
                        }
                    }
                    if (anyLost)
                        this->m_runtimeIdentificationTimer -= 10; // delay the timer if there are lost robot (otherwise as soon it has been found it starts)
                    else
                        runRTI = true;
                }
                if (runRTI) runtimeIdentify();
            }
            break;
        }
        case IDENTIFY:{
            this->identifyKilobots();
            break;
        }
        default:{
            this->trackKilobots();
            break;
        }
        }

        ++time;

        if (time % 5 == 0) {
            float c_time = float(this->timer.elapsed())/1000.0f;
            emit errorMessage(QString("FPS = ") + QString::number(5.0f/(c_time-last_time)));
            last_time = c_time;
        }

    }

}

void KilobotTracker::runtimeIdentify(){
    //qDebug() << "Runtime-ID: " << this->m_runtimeIdentificationTimer;
    if (!this->m_ongoingRuntimeIdentification) {
        qDebug() << "Runtime-ID: started for robots" << this->pendingRuntimeIdentification;

        emit setRuntimeIdentificationLock(true);
        this->m_ongoingRuntimeIdentification = true;
        this->runtimeIDtimer.start();

        // Reset lists and counters
        this->m_runtimeIdentificationTimer = 0;
        this->foundIDs.clear();
        this->currentID = 0;

        // broadcast ID
        identifyKilobot(this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID(), true);
        qDebug() << "Runtime-ID: Signalled ID" << this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID();
    }

    //#ifdef USE_CUDA
    //    Mat display;
    //    this->finalImageB.download(display);
    //    cv::cvtColor(display, display, CV_GRAY2RGB);
    //#else
    //    Mat display;
    //    cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);
    //#endif
    //    this->getKiloBotLights(display);

    if(this->m_runtimeIdentificationTimer++ >= 10) {
        this->m_runtimeIdentificationTimer = 0;
        int blueBots = 0;
        int bot = -1;

        // I allow only swaps between robots on the pending list, if there is some false detect I update the list
        for (int i = 0; i < this->pendingRuntimeIdentification.size(); ++i) {
            //            if (this->kilos[this->pendingRuntimeIdentification[i]]->getLedColour() == BLUE) {
            if (this->lost_count[this->pendingRuntimeIdentification[i]] < 10 && this->kilos[this->pendingRuntimeIdentification[i]]->getLedColour() == BLUE) {
                qDebug() << "Runtime-ID: found blue LED robot #" << this->kilos[this->pendingRuntimeIdentification[i]]->getID();
                blueBots++;
                bot = this->pendingRuntimeIdentification[i];
            }
        }

        if (blueBots == 1){
            //kilos[bot]->setID((uint8_t) currentID);
            //this->circsToDraw.push_back(drawnCircle {Point(kilos[bot]->getPosition().x(),kilos[bot]->getPosition().y()), 4, QColor(0,255,0), 2, ""});
            this->foundIDs.push_back(this->currentID);
            qDebug() << "Looking for position " << this->pendingRuntimeIdentification[this->currentID] << " (id:"<<this->kilos[this->pendingRuntimeIdentification[currentID]]->getID()<<")"<<
                        "and found position" << bot << " (id:"<<this->kilos[bot]->getID()<<")";
            if (this->pendingRuntimeIdentification[this->currentID] == bot) {
                qDebug() << "Runtime-ID: Kilobot #" << this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID() << "was correctly assigned";
            } else {
                // swapping robot positions (otherwise I lose the robot tracking)
                qDebug() << "Runtime-ID: reassignment!! Kilobot #" << this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID() << "moved to new location.";
                QPointF tmpPos = this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getPosition();
                QPointF tmpVel = this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getVelocity();
                kilobot_colour tmpCol = this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getLedColour();
                this->kilos[this->pendingRuntimeIdentification[this->currentID]]->updateState(
                            QPointF(this->kilos[bot]->getPosition().x(),this->kilos[bot]->getPosition().y()),
                            this->kilos[bot]->getVelocity(),this->kilos[bot]->getLedColour() );
                this->kilos[bot]->updateState( tmpPos, tmpVel, tmpCol );

                // re-upload locations on GPU
                Mat tempKbLocs(1,this->kilos.size(), CV_32FC2);
                float * data = (float *) tempKbLocs.data;
                for (int i = 0; i < this->kilos.size(); ++i) {
                    data[i*2] = this->kilos[i]->getPosition().x();
                    data[i*2+1] = this->kilos[i]->getPosition().y();
                }
                this->kbLocs.upload(tempKbLocs);
            }
        } else if (blueBots > 1) {
            qDebug() << "Runtime-ID: Multiple detections. ID n." << this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID() << "has not been re-assigned";
        } else if (blueBots < 1) {
            qDebug() << "Runtime-ID: No bot found for ID n." << this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID() << "in the pending list";
            blueBots = 0;
            bot = -1;
            for (int i = 0; i < this->kilos.size(); ++i) {
                if (this->kilos[i]->getLedColour() == BLUE) {
                    blueBots++;
                    bot = i;
                    qDebug() << "Runtime-ID: Found robot #" << kilos[i]->getID() << "with blue LED. Added to pending for runtime identification";
                    if (!this->pendingRuntimeIdentification.contains(i)) this->pendingRuntimeIdentification.push_back(i);
                }
            }
            if (!blueBots) qDebug() << "No Blue response from all the swarm.";
        }

        if (this->foundIDs.size() == this->pendingRuntimeIdentification.size() ) { // all robots has been found
            // || currentID == this->pendingRuntimeIdentification.size()-2 ) {     OR I looped through all
            qDebug() << "Runtime-ID: All" << this->foundIDs.size() << "robots have been correcly runtime-identified.";
            // clearing pending list
            this->pendingRuntimeIdentification.clear();
            this->foundIDs.clear();
            this->m_runtimeIdentificationTimer = 0;
            // stopping the runtime identification process
            this->m_ongoingRuntimeIdentification = false;
            // inform the experiment the runtime ID has stopped
            emit setRuntimeIdentificationLock(false);
            // inform the kilobots
            identifyKilobot(-1, true);
        }
        else { // next id
            this->currentID++;
            while( this->foundIDs.contains(this->currentID) || this->currentID >= this->pendingRuntimeIdentification.size() ){
                if (++this->currentID >= this->pendingRuntimeIdentification.size()) this->currentID = 0;
            }
            identifyKilobot(this->kilos[this->pendingRuntimeIdentification[this->currentID]]->getID(), true);
            qDebug() << "Runtime-ID: Signalled ID" << this->kilos[this->pendingRuntimeIdentification[currentID]]->getID();
            // setting maximum time allowed for runtime ID identification
            //int maxTimeForRuntimeID = (this->pendingRuntimeIdentification.size() < 6)? 10000 : this->pendingRuntimeIdentification.size() * 2000;
            int maxTimeForRuntimeID = qMax<int>(30000,1000*5*this->pendingRuntimeIdentification.size());
            // check if I am trying identifying on lost robots
            for (int i = 0; i < this->pendingRuntimeIdentification.size(); ++i) {
                /* if a missing robot has been lost for more than 10 step, the runtime identification is stopped but pending list only partially cleared */
                /* also stop if the process is running for more than 10s */
                if (!this->foundIDs.contains(i) && this->lost_count[this->pendingRuntimeIdentification[i]] > 10 || this->runtimeIDtimer.elapsed() > maxTimeForRuntimeID ){
                    qDebug() << "Runtime-ID: Giving up runtime identification because involves lost robots or it's taking too much time, e.g. robot #" << this->kilos[this->pendingRuntimeIdentification[i]]->getID();
                    this->foundIDs.clear();
                    this->m_runtimeIdentificationTimer = -50;
                    // stopping the runtime identification process
                    this->m_ongoingRuntimeIdentification = false;
                    // inform the experiment the runtime ID has stopped
                    emit setRuntimeIdentificationLock(false);
                    // inform the kilobots
                    identifyKilobot(-1, true);
                }
            }
        }
    }

    //this->drawOverlay(display);

    //this->showMat(display);
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

    if (this->finalImageB.empty()) return;

    Mat res2;
    Mat display;

#ifdef USE_CUDA
    this->finalImageB.download(display);
    display.copyTo(res2);
#else
    this->finalImage.copyTo(display);
    res2 = this->finalImage;
#endif

    vector<Vec3f> circles;
    HoughCircles(res2,circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/ \
                 ,this->kbMaxSize-1/* circle distance*/ \
                 ,cannyThresh /* Canny threshold*/ \
                 ,houghAcc /*cicle algorithm accuracy*/ \
                 ,kbMinSize/* min circle size*/ \
                 ,kbMaxSize/* max circle size*/);

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
        putText(display, this->showIDs?to_string(i):"", center, FONT_HERSHEY_PLAIN, 2.7, Scalar(0,0,255), 3);
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
    emit errorMessage(QString::fromStdString(to_string(kilos.size()))+ QString(" kilobots found!"));
    emit activateExpButtons(this->kilos.size());
}

void KilobotTracker::identifyKilobots()
{

    if (this->kilos.isEmpty()){
        qDebug() << "There are no Kilobots to be indetified. Stopping the operation";
        LOOPstartstop(IDENTIFY);
        return;
    }

#ifdef USE_CUDA
    Mat display;
    this->finalImageB.download(display);
    cv::cvtColor(display, display, CV_GRAY2RGB);
#else
    Mat display;
    cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);
#endif

    if (time == 0)
    {
        qDebug() << "Max ID to test is" << maxIDtoCheck;

        // check that max id is higher than number of tracked kilobots
        if (maxIDtoCheck < qMax(0,kilos.size()-2)){
            qDebug() << "ERROR! More robots than possible IDs. Change the max ID to test (currently it's" << maxIDtoCheck << ")";
            return;
        }

        // Reset lists and counters
        currentID = 0;
        foundIDs.clear();
        assignedCircles.clear();
        this->circsToDraw.clear();
        this->linesToDraw.clear();

        // broadcast ID
        identifyKilobot(currentID);
        qDebug() << "Try ID" << currentID;
    }

    // Restart from current ID=0 if we reached MaxIDtoCheck
    if (currentID > maxIDtoCheck){
        currentID = 0;
        while( foundIDs.contains(currentID) ){
            currentID++;
        }
        // broadcast ID
        identifyKilobot(currentID);
        qDebug() << "Try ID" << currentID;
    }
    // broadcast ID
    //identifyKilobot(currentID);

    this->getKiloBotLights(display);

    if(time % 5 == 4)
    {
        int blueBots = 0;
        int bot = -1;
        for (uint i = 0; i < (uint) kilos.size(); ++i) {
            if (kilos[i]->getLedColour() == BLUE) {
                qDebug() << "Found ID" << currentID;
                blueBots++;
                bot = i;
            }
        }

        if (blueBots == 1 && !assignedCircles.contains(bot)){
            kilos[bot]->setID((uint8_t) currentID);
            this->circsToDraw.push_back(drawnCircle {Point(kilos[bot]->getPosition().x(),kilos[bot]->getPosition().y()), 4, QColor(0,255,0), 2, ""});
            foundIDs.push_back(currentID);
            assignedCircles.push_back(bot);
            qDebug() << "Success!! ID n." << currentID << " successfully assigned!!";
        } else if (blueBots > 1) {
            qDebug() << "Multiple detections. ID n." << currentID << " has not been assigned";
        } else if (blueBots < 1) {
            qDebug() << "No bot found for ID n." << currentID;
        } else if (blueBots == 1 && assignedCircles.contains(bot)) {
            qDebug() << "Trying to re-assign the same circle to a second ID. I don't make this assigment and I undo the previous, i.e., ID" << kilos[bot]->getID();
            foundIDs.removeOne( kilos[bot]->getID() );
            assignedCircles.removeOne(bot);
        }

        if (foundIDs.size() == kilos.size()) { // all robots has been found
            qDebug() << "All" << kilos.size() << "robots have been correcly identified. Well Done, mate! Now, it's time for serious stuff.";
            this->LOOPstartstop(IDENTIFY);
        }
        else { // next id
            ++currentID;
            while( foundIDs.contains(currentID) ){
                currentID++;
            }
            identifyKilobot(currentID);
            if (currentID <= maxIDtoCheck) qDebug() << "Try ID" << currentID;
        }
    }

    this->drawOverlay(display);

    this->showMat(display);
    //this->showMat(this->finalImageCol);
}

void KilobotTracker::identifyKilobot(int id, bool runtime = false){
    // decompose id
    QVector < uint8_t > data(9);
    data[0] = id >> 8;
    data[1] = id & 0xFF;

    kilobot_broadcast msg;
    if (runtime) {
        msg.type = 119;
    } else {
        msg.type = 120;
    }
    msg.data = data;
    emit this->broadcastMessage(msg);
}

void KilobotTracker::identifyKilobot(int id)
{
    // decompose id
    QVector < uint8_t > data(9);
    data[0] = id >> 8;
    data[1] = id & 0xFF;

    kilobot_broadcast msg;
    msg.type = 120;
    msg.data = data;
    emit this->broadcastMessage(msg);

}

QString type2str(int type) {
    QString r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}
void KilobotTracker::trackKilobots()
{

    // convert for display
#ifdef USE_CUDA
    Mat display;
    Mat temp_for_reacquire;
    this->finalImageB.download(temp_for_reacquire);
    cv::cvtColor(temp_for_reacquire, display, CV_GRAY2RGB);
#else
    Mat display;
    cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);
#endif

    switch (this->trackType) {
    {
    case NO_TRACK:
            this->showMat(display);
            return;
    }
    {
    case CIRCLES_NAIVE:

            if (this->kilos.size() == 0) break;

            int circle_acc = this->houghAcc;
            cuda::GpuMat circlesGpu;

            vector < cuda::GpuMat > circChans;
            vector < cuda::GpuMat > kbChans;

            this->hough->setVotesThreshold(circle_acc);
            this->hough->detect(this->finalImageB,circlesGpu,stream);

            //***************************************************************
            // FOR DEGUB: draw all the circles found through GPU-hough
            //            vector<Vec3f> circles;
            //            circlesGpu.download(circles);
            //            for( size_t i = 0; i < circles.size(); i++ )
            //            {
            //                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            //                int radius = cvRound(circles[i][2]);
            //                // draw the circle outline
            //                circle( display, center, radius, Scalar(50,255,255), 1, 8, 0 );
            //            }
            //***************************************************************

            // get the channels so we can get rid of the sizes and use locations only
            cuda::split(circlesGpu,circChans);
            cuda::split(kbLocs,kbChans);

            //#define DEBUG_TRACKING true
            Mat xCpu;
            Mat yCpu;
            circChans[0].download(xCpu);
            circChans[1].download(yCpu);
#ifdef DEBUG_TRACKING
            Mat szCpu;
            circChans[2].download(szCpu);

            for (int i = 0; i < xCpu.size().width;++i) {
                cv::circle(display,Point(mean(xCpu(Rect(i,0,1,1)))[0],mean(yCpu(Rect(i,0,1,1)))[0]),mean(szCpu(Rect(i,0,1,1)))[0],Scalar(0,255,0),2);
            }
#endif

            // create ones
            cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
            cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

            // expanded mats
            cuda::GpuMat all_x_c;
            cuda::GpuMat all_y_c;
            cuda::GpuMat all_x_kb;
            cuda::GpuMat all_y_kb;

            QVector <QPointF> previousPositions;
            previousPositions.resize(this->kilos.size());

            for (int i = 0; i < this->kilos.size(); ++i) {
                // store the previous position
                previousPositions[i] = this->kilos[i]->getPosition();
            }

            if (circChans[0].size().width  > 0) {


                // expand circle x's & y's
                vector < cuda::Stream > streams(4);
                cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
                cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

                // expand kb x's & y's
                cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
                cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

                // diffs
                cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
                cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

                // distances
                cuda::magnitude(all_x_c,all_y_c,all_x_c);

                double * min = new double;
                Point * minLoc = new Point();

                Mat localDists;

                all_x_c.download(localDists);

                //cout << endl << localDists << endl;

                // download circChans
                Mat circChansXCpu;
                Mat circChansYCpu;

                circChans[0].download(circChansXCpu);
                circChans[1].download(circChansYCpu);

                QMap <int,int> previously_lost;
                //                previous_lost_count.resize(this->lost_count.size());
                //                for (int i=0; i<this->lost_count.size(); ++i){
                //                    previous_lost_count[i] = this->lost_count[i];
                //                }

                // assign the new position of each kilobot as the new closest detected circle (if within a small distance from previous pos)
                for (int i = 0; i < this->kilos.size(); ++i) {
                    // find the closest circle (this operation is much quicker on the CPU than on GPU)
                    minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                    //cv::cuda::minMaxLoc(all_x_c(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                    if (this->lost_count[i]>0) previously_lost[i] = this->lost_count[i];
                    // work out if we should update...
                    if (*min < float(this->kbMinSize)/1.15 && this->lost_count[i] < 10) { // check if the distance between the old and new position is small enough
                        circChans[0](Rect((*minLoc).y,0,1,1)).copyTo(kbChans[0](Rect(i,0,1,1)));
                        circChans[1](Rect((*minLoc).y,0,1,1)).copyTo(kbChans[1](Rect(i,0,1,1)));
                        //cout << endl << circChansXCpu << endl;
                        this->lost_count[i] = 0;
                        // and on the cpu
                        kilos[i]->updateState(QPointF(circChansXCpu.at<float>((*minLoc).y),circChansYCpu.at<float>((*minLoc).y)),kilos[i]->getVelocity(), kilos[i]->getLedColour());
                    } else {

                        // we could not assign a circle onto the kilobot! maybe we lost the tag?

                        this->lost_count[i]++;
                        kilobot_id kID = kilos[i]->getID();
                        if (this->lost_count[i]>5){
                            qDebug() << "Lost tag #" << kID << "!! (count is " << this->lost_count[i] << ")";
                            cv::circle(display,Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()),15,Scalar(255,0,255),5);
                        }

                        Mat kbLocsCpuX;
                        Mat kbLocsCpuY;

                        kbChans[0].download(kbLocsCpuX);
                        kbChans[1].download(kbLocsCpuY);

                        /*** if the area to search is small enough it might quicker to crop the image, find again cirlces and loop over a smaller set ***/
                        if (this->lost_count[i] > 10 && this->lost_count[i] <= 150) {

                            Rect bb = this->getKiloBotBoundingBox(i, this->lost_count[i]*5.0/this->kbMaxSize);
#ifdef USE_CUDA
                            cuda::GpuMat temp;
#else
                            Mat temp[3];
#endif
                            // switch cam/vid source depending on position...
                            //                            if (bb.x < 2000/2 && bb.y < 2000/2) {
                            //                                Rect bb_adj = bb;
                            //                                bb_adj.x = bb_adj.x +100;
                            //                                bb_adj.y = bb_adj.y +100;
                            //                                //for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[0]][c](bb_adj);
                            //                                temp = this->fullImages[clData.inds[0]][0](bb_adj);
                            //                            } else if (bb.x > 2000/2-1 && bb.y < 2000/2) {
                            //                                Rect bb_adj = bb;
                            //                                bb_adj.x = bb_adj.x -900;
                            //                                bb_adj.y = bb_adj.y +100;
                            //                                //for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[1]][c](bb_adj);
                            //                                temp = this->fullImages[clData.inds[1]][0](bb_adj);
                            //                            } else if (bb.x < 2000/2 && bb.y > 2000/2-1) {
                            //                                Rect bb_adj = bb;
                            //                                bb_adj.x = bb_adj.x +100;
                            //                                bb_adj.y = bb_adj.y -900;
                            //                                //for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[2]][c](bb_adj);
                            //                                temp = this->fullImages[clData.inds[2]][0](bb_adj);
                            //                            } else if (bb.x > 2000/2-1 && bb.y > 2000/2-1) {
                            //                                Rect bb_adj = bb;
                            //                                bb_adj.x = bb_adj.x -900;
                            //                                bb_adj.y = bb_adj.y -900;
                            //                                //for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[3]][c](bb_adj);
                            //                                temp = this->fullImages[clData.inds[3]][0](bb_adj);
                            //                            }
                            temp = this->finalImageB(bb);

                            std::vector<cv::Vec3f> circles_rematch;

#ifdef USE_CUDA
                            cuda::GpuMat circlesGpu_temp;
                            this->hough->setVotesThreshold(this->houghAcc);
                            this->hough->detect(temp,circlesGpu_temp,stream);
                            if (circlesGpu_temp.size().width > 0) circlesGpu_temp.download(circles_rematch);
#else
                            // find circles
                            HoughCircles(temp[0],circles_rematch,CV_HOUGH_GRADIENT,1.0,1.0,this->cannyThresh,this->houghAcc,this->kbMinSize,this->kbMaxSize);
#endif

                            //                            qDebug() << "circ size: " << circles_rematch.size();
                            //                            this->circsToDraw.push_back(drawnCircle {Point(bb.x,bb.y), 4, QColor(0,255,0), 2, ""});
                            //                            this->circsToDraw.push_back(drawnCircle {Point(qMin(this->finalImageB.size().width,bb.x+bb.width),bb.y), 4, QColor(0,255,0), 2, ""});
                            //                            this->circsToDraw.push_back(drawnCircle {Point(qMin(this->finalImageB.size().width,bb.x+bb.width),qMin(this->finalImageB.size().height,bb.y+bb.height)), 4, QColor(0,255,0), 2, ""});
                            //                            this->circsToDraw.push_back(drawnCircle {Point(bb.x,qMin(this->finalImageB.size().height,bb.y+bb.height)), 4, QColor(0,255,0), 2, ""});

                            for (uint c = 0; c < circles_rematch.size(); ++c) {
                                int cur_x = bb.x+cvRound(circles_rematch[c][0]);
                                int cur_y = bb.y+cvRound(circles_rematch[c][1]);

                                bool foundOverlapWithOtherKilobot = false;
                                for (int k = 0; k < kilos.size(); ++k) {
                                    // check if within small distance to a KB
                                    if (qAbs(kilos[k]->getPosition().x()-cur_x) < (this->kbMinSize*1.5) && qAbs(kilos[k]->getPosition().y()-cur_y) < (this->kbMinSize*1.5)) { // it was 16
                                        foundOverlapWithOtherKilobot = true;
                                        break;
                                    }
                                }
                                if (!foundOverlapWithOtherKilobot) {
                                    qDebug() << "Lost bot #" << kID << "has been Found through local search";
                                    // pair up on CPU!
                                    kilos[i]->updateState(QPointF(cur_x,cur_y),kilos[i]->getVelocity(),kilos[i]->getLedColour());
                                    // and GPU
                                    kbLocsCpuX(Rect(i,0,1,1)) = cur_x;
                                    kbLocsCpuY(Rect(i,0,1,1)) = cur_y;

                                    kbChans[0].upload(kbLocsCpuX);
                                    kbChans[1].upload(kbLocsCpuY);

                                    this->circsToDraw.push_back(drawnCircle {Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()), 4, QColor(0,255,0), 2, ""});
                                    this->lost_count[i] = 0;
                                    break;
                                }
                            }

                        } else if (this->lost_count[i] > 10) { /* go through circles on full image */

                            // go through xCpu and yCpu (circle locs) and compare to KB locations
                            // Point(mean(xCpu(Rect(i,0,1,1)))[0],mean(yCpu(Rect(i,0,1,1)))[0])
                            for (int c = 0; c < xCpu.size().width; ++c)
                            {
                                int cur_x = mean(xCpu(Rect(c,0,1,1)))[0];
                                int cur_y = mean(yCpu(Rect(c,0,1,1)))[0];

                                bool foundOverlapWithOtherKilobot = false;
                                for (int k = 0; k < kilos.size(); ++k) {
                                    // check if within small distance to a KB
                                    if (qAbs(kilos[k]->getPosition().x()-cur_x) < (this->kbMinSize*1.5) && qAbs(kilos[k]->getPosition().y()-cur_y) < (this->kbMinSize*1.5)) { // it was 16
                                        foundOverlapWithOtherKilobot = true;
                                        break;
                                    }
                                }
                                if (!foundOverlapWithOtherKilobot) {
                                    // check it is in a sane distance
                                    if (qAbs(kilos[i]->getPosition().x()-cur_x) < this->lost_count[i]*5 && qAbs(kilos[i]->getPosition().y()-cur_y) < this->lost_count[i]*5) {
                                        qDebug() << "Lost bot #" << kID << "has been Found through Global search";
                                        // pair up on CPU!
                                        kilos[i]->updateState(QPointF(cur_x,cur_y),kilos[i]->getVelocity(),kilos[i]->getLedColour());
                                        // and GPU
                                        kbLocsCpuX(Rect(i,0,1,1)) = cur_x;
                                        kbLocsCpuY(Rect(i,0,1,1)) = cur_y;

                                        kbChans[0].upload(kbLocsCpuX);
                                        kbChans[1].upload(kbLocsCpuY);

                                        this->circsToDraw.push_back(drawnCircle {Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()), 4, QColor(0,255,0), 2, ""});
                                        this->lost_count[i] = 0;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                /* undo the update if the new position is overlapping with another kilobot */
                for (int i = 0; i < this->kilos.size()-1; ++i) {
                    for (int j = i+1; j < this->kilos.size(); ++j) {
                        /* if the positions are too close */
                        if (qAbs(kilos[i]->getPosition().x()-kilos[j]->getPosition().x()) < (this->kbMinSize) && qAbs(kilos[i]->getPosition().y()-kilos[j]->getPosition().y()) < (this->kbMinSize)) {

                            // need to download the full chans to update them (to check if instead it's quicker to re-upload the full kbLocs afterwards)
                            Mat kbLocsCpuX;
                            Mat kbLocsCpuY;
                            kbChans[0].download(kbLocsCpuX);
                            kbChans[1].download(kbLocsCpuY);

                            int which_revert = 0; // 0 don't know, 1 revert robot[i] to its previous position, 2 revert robot[j] to its previous position

                            // whatever happens runtime-identification is triggered
                            if (!this->pendingRuntimeIdentification.contains(i)) this->pendingRuntimeIdentification.push_back(i);
                            if (!this->pendingRuntimeIdentification.contains(j)) this->pendingRuntimeIdentification.push_back(j);

                            /* if one tag has been lost (i.e. lost_count[_] > 0) it has lower priority and stays where it was
                             * The motivation is that we want to avoid that a lost tag (which remains somewhere) is picked up by another robot passing by */
                            //if (previous_lost_count[i] > 0 && previous_lost_count[j] == 0) {
                            if (previously_lost.contains(i) && !previously_lost.contains(j)) {
                                this->lost_count[i] = previously_lost[i];
                                which_revert = 1;
                            } else if (!previously_lost.contains(i) && previously_lost.contains(j)) {
                                this->lost_count[j] = previously_lost[j];
                                which_revert = 2;
                            } else
                                /* If none was lost (both it's impossible), I move back the robot that has moved more in the last timestep */
                                if (  (pow(kilos[i]->getPosition().x()-previousPositions[i].x(),2)+pow(kilos[i]->getPosition().y()-previousPositions[i].y(),2))
                                      >
                                      (pow(kilos[j]->getPosition().x()-previousPositions[j].x(),2)+pow(kilos[j]->getPosition().y()-previousPositions[j].y(),2)) ){
                                    which_revert = 1;
                                } else {
                                    which_revert = 2;
                                }

                            switch(which_revert){
                            case 1:{
                                qDebug() << "Reverted swap ("<<kilos[i]->getID()<<"/"<<kilos[j]->getID()<<") of tag #" << kilos[i]->getID() << " over robot #" << kilos[j]->getID() << ". Keep an eye on that area!";
                                //qDebug() << "Reverted swap ("<<kilos[i]->getID()<<"/"<<kilos[j]->getID()<<") of tag #" << kilos[i]->getID() << " ("<< kilos[i]->getPosition()<<", goes to pos:" <<previousPositions[i]<< ") over robot #" << kilos[j]->getID() << " ("<< kilos[j]->getPosition()<<"). Keep an eye on that area!";
                                // update CPU
                                kilos[i]->updateState(QPointF(previousPositions[i].x(),previousPositions[i].y()),kilos[i]->getVelocity(), kilos[i]->getLedColour());
                                // and GPU
                                kbLocsCpuX(Rect(i,0,1,1)) = previousPositions[i].x();
                                kbLocsCpuY(Rect(i,0,1,1)) = previousPositions[i].y();

                                this->lost_count[i]++;
                                cv::circle(display,Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()),10,Scalar(0,255,255),5);
                                cv::circle(display,Point(kilos[j]->getPosition().x(),kilos[j]->getPosition().y()),10,Scalar(0,255,255),5);
                                break;
                            }
                            case 2:{
                                qDebug() << "Reverted swap ("<<kilos[j]->getID()<<"/"<<kilos[i]->getID()<<") of tag #" << kilos[j]->getID() << " over robot #" << kilos[i]->getID() << ". Keep an eye on that area!";
                                //qDebug() << "Reverted swap ("<<kilos[j]->getID()<<"/"<<kilos[i]->getID()<<") of tag #" << kilos[j]->getID() << " ("<< kilos[j]->getPosition()<<", goes to pos:" <<previousPositions[j]<< ") over robot #" << kilos[i]->getID() << " ("<< kilos[i]->getPosition()<<"). Keep an eye on that area!";
                                // update CPU
                                kilos[j]->updateState(QPointF(previousPositions[j].x(),previousPositions[j].y()),kilos[j]->getVelocity(), kilos[j]->getLedColour());
                                // and GPU
                                kbLocsCpuX(Rect(j,0,1,1)) = previousPositions[j].x();
                                kbLocsCpuY(Rect(j,0,1,1)) = previousPositions[j].y();

                                this->lost_count[j]++;
                                cv::circle(display,Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()),15,Scalar(0,255,255),5);
                                cv::circle(display,Point(kilos[j]->getPosition().x(),kilos[j]->getPosition().y()),15,Scalar(0,255,255),5);
                                break;
                            }
                            default:{ // I shouldn't arrive here
                                qDebug() << "WARNING! Bug code in robot tracking, an unforecasted case has been triggered after tag-swapping.";
                                break;
                            }
                            }

                            kbChans[0].upload(kbLocsCpuX);
                            kbChans[1].upload(kbLocsCpuY);
                        }
                    }
                }

                // recompose kbLocs
                cuda::merge(kbChans,kbLocs);
            }

            // now we must do the LED detection:
            if (this->t_type & LED || this->t_type & ADAPTIVE_LED) {
                this->getKiloBotLights(display);
            }

            // getting the orientation
            if (this->t_type & ROT){
                //float smooth_fact = 0.5;
                for (int i = 0; i < this->kilos.size(); ++i) {
                    //                        int new_x = float(kilos[i]->getPosition().x())*(1.0-smooth_fact) + float(previousPositions[i].x())*smooth_fact;
                    //                        int new_y = float(kilos[i]->getPosition().y())*(1.0-smooth_fact) + float(previousPositions[i].y())*smooth_fact;

                    //                        // update velocity
                    //                        QPointF prevVel = this->kilos[i]->getVelocity();
                    //                        QPointF newSmoothPos = QPointF(new_x,new_y);

                    //                        QPointF newVel = prevVel*(2.0f/3.0f) + (newSmoothPos - previousPositions[i])*(1.0f/3.0f);

                    this->kilos[i]->posBuffer.addPosition(this->kilos[i]->getPosition());
                    QPointF newVel = this->kilos[i]->posBuffer.getOrientationFromPositions();

                    //                    if (this->t_type & LED || this->t_type & ADAPTIVE_LED) {

                    //                        QLineF lightLine = QLineF(QPointF(0,0),QPointF(light.pos.x,light.pos.y));
                    //                        QLineF velLine = QLineF(QPointF(0,0),newVel);

                    //                        // if we have a light
                    //                        if (light.col != OFF) {

                    //                            if (velLine.length() < 1.0f) {
                    //                                lightLine.setLength(0.9f);
                    //                                lightLine.setAngle(lightLine.angle() + 20.0f);
                    //                                newVel = lightLine.p2();
                    //                            } else {
                    //                                // combine LED and velocity estimates
                    //                                lightLine.setLength(velLine.length());
                    //                                // align to forward
                    //                                lightLine.setAngle(lightLine.angle() + 20.0f);
                    //                                newVel = (lightLine.p2() + velLine.p2())*0.5f;
                    //                            }

                    //                        }
                    //                    }
                    kilos[i]->velocityBuffer.addOrientation(newVel);
                    kilos[i]->updateState(kilos[i]->getPosition(), kilos[i]->velocityBuffer.getAvgOrientation(), kilos[i]->getLedColour());
                }
            }

            // we add overlay circles and orientation */
            for (int i = 0; i < this->kilos.size(); ++i) {
                //cv::circle(display,Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()),10,Scalar(0,255,0),2);
                Scalar rgbColor(200,200,0);
                switch (kilos[i]->getLedColour()){
                case OFF:{
                    break;
                }
                case RED:{
                    rgbColor[0] = 255;
                    rgbColor[1] = 0;
                    rgbColor[2] = 0;
                    break;
                }
                case GREEN:{
                    rgbColor[0] = 0;
                    rgbColor[1] = 255;
                    rgbColor[2] = 0;
                    break;
                }
                case BLUE:{
                    rgbColor[0] = 0;
                    rgbColor[1] = 0;
                    rgbColor[2] = 255;
                    break;
                }
                }
                cv::circle(display,Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()),10,rgbColor,2);

                if (this->t_type & ROT){
                    Point center(round(kilos[i]->getPosition().x()), round(kilos[i]->getPosition().y()));
                    QLineF currVel = QLineF(QPointF(0,0),this->kilos[i]->getVelocity());
                    currVel.setLength(currVel.length()*10.0f+20.0f);
                    QPointF hdQpt = currVel.p2() + this->kilos[i]->getPosition();
                    Point heading(hdQpt.x(), hdQpt.y());
                    line(display, center, heading, rgbColor, 3);
                }

                //qDebug() << "Single vel is" << this->kilos[i]->getVelocity() << "AVG vel is" << this->kilos[i]->velocityBuffer.getAvgOrientation();

                if (this->showIDs) {
                    cv::putText(display, QString::number(this->kilos[i]->getID()).toStdString(), Point(kilos[i]->getPosition().x(),kilos[i]->getPosition().y()), FONT_HERSHEY_PLAIN, 2.7, rgbColor, 3);
                }
            }
            this->drawOverlay(display);

            break;
    }
    {
    case CIRCLES_LOCAL:

            // setup the tracking region around each KB's last known position
            float maxDist = 1.2f*this->kbMaxSize;

            vector < Rect > bbs;

            // get the bounding box info for all KB
            for (uint i = 0; i < (uint) this->kilos.size(); ++i) {
                bbs.push_back(this->getKiloBotBoundingBox(i, 1.2f));
            }


            for (uint i = 0; i < (uint) this->kilos.size(); ++i) {


                Rect bb = bbs[i];
#ifdef USE_CUDA
                cuda::GpuMat temp[3];
#else
                Mat temp[3];
#endif
                // setup temp pars with previous KB state
                kiloLight light;
                light.col = kilos[i]->getLedColour();
                QPointF newPos = this->kilos[i]->getPosition();
                QPointF newVel = this->kilos[i]->getVelocity();

                // track light
                if (this->t_type & LED || this->t_type & ADAPTIVE_LED) {

                    // switch cam/vid source depending on position...
                    if (bb.x < 2000/2 && bb.y < 2000/2) {
                        Rect bb_adj = bb;
                        bb_adj.x = bb_adj.x +100;
                        bb_adj.y = bb_adj.y +100;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[0]][c](bb_adj);
                    } else if (bb.x > 2000/2-1 && bb.y < 2000/2) {
                        Rect bb_adj = bb;
                        bb_adj.x = bb_adj.x -900;
                        bb_adj.y = bb_adj.y +100;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[1]][c](bb_adj);
                    } else if (bb.x < 2000/2 && bb.y > 2000/2-1) {
                        Rect bb_adj = bb;
                        bb_adj.x = bb_adj.x +100;
                        bb_adj.y = bb_adj.y -900;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[2]][c](bb_adj);
                    } else if (bb.x > 2000/2-1 && bb.y > 2000/2-1) {
                        Rect bb_adj = bb;
                        bb_adj.x = bb_adj.x -900;
                        bb_adj.y = bb_adj.y -900;
                        for (uint c = 0; c < 3; ++c) temp[c] = this->fullImages[clData.inds[3]][c](bb_adj);
                    }

                    if (this->t_type & ADAPTIVE_LED) {
                        light = this->getKiloBotLightAdaptive(temp, Point(bb.width/2,bb.height/2),i);
                    } else if (this->t_type & LED){
                        light = this->getKiloBotLight(temp, Point(bb.width/2,bb.height/2),i);
                    }

                }


                if (this->t_type & POS) {

                    bool no_match = true;
                    int circle_acc = 30;

                    std::vector<cv::Vec3f> circles;
                    Mat circlesCpu;

                    while (no_match && circle_acc > 10) {

#ifdef USE_CUDA
                        cuda::GpuMat circlesGpu;

                        this->hough->setVotesThreshold(circle_acc);
                        this->hough->detect(temp[0],circlesGpu,stream);

                        circlesGpu.download(circlesCpu);

                        circles.assign((float*)circlesCpu.datastart, (float*)circlesCpu.dataend);


#else
                        // try fixed pars
                        int minS = this->kbMinSize;//9;
                        int maxS = this->kbMaxSize;//25;

                        HoughCircles(temp[0],circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/,1.0/*maxS-1.0 circle distance*/,this->cannyThresh /* Canny threshold*/,circle_acc /*cicle algorithm accuracy*/,minS/* min circle size*/,maxS/* max circle size*/);
#endif

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

                    // FIND ISSUES
                    if (time % 10 == 0 && false) {
                        // we want to find tags that haven't moved, and tags too close together

                        QVector < indexPair > closeIndexPairs;

                        // too close
                        for (int i = 0; i < this->kilos.size(); ++i) {
                            for (int j = i; j < kilos.size(); ++j) {
                                QLineF dist(kilos[i]->getPosition(), kilos[j]->getPosition());
                                if (dist.length() < this->kbMinSize-2.0f) {
                                    closeIndexPairs.push_back(indexPair{i,j});
                                }
                            }
                        }

                        QVector < int > notMovedIndices;

                        // not moved
                        //for (int i = 0; i < this->kilos.size(); ++i) {

                        if (closeIndexPairs.length() > 0 || notMovedIndices.length() > 0) {

                            // re-acquire

                            // pause experiment

                            // identify kilobots


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
    //this->showMat(finalImageCol);

}



void KilobotTracker::drawOverlay(Mat & display)
{
    QVector < drawnCircle > alphaCircles;
    for (int i = 0; i < this->circsToDraw.size(); ++i) {

        if (this->circsToDraw[i].transparent) {
            alphaCircles.push_back(this->circsToDraw[i]);
        } else{
            cv::circle(display,this->circsToDraw[i].pos, this->circsToDraw[i].r,
                       Scalar(this->circsToDraw[i].col.red(),this->circsToDraw[i].col.green(),this->circsToDraw[i].col.blue()),
                       this->circsToDraw[i].thickness);

            if (!this->circsToDraw[i].text.empty()){
                cv::putText(display, this->circsToDraw[i].text,
                            this->circsToDraw[i].pos+Point(-15,10) , //+Point(this->circsToDraw[i].r,-this->circsToDraw[i].r),
                            FONT_HERSHEY_DUPLEX, 1,
                            Scalar(this->circsToDraw[i].col.red(),this->circsToDraw[i].col.green(),this->circsToDraw[i].col.blue()), 2, 8);
            }
        }
    }
    QVector < drawnLine > alphaLines;
    for (int i = 0; i < this->linesToDraw.size(); ++i) {

        if (this->linesToDraw[i].transparent) {
            alphaLines.push_back(this->linesToDraw[i]);
        } else{
            cv::polylines(display, this->linesToDraw[i].pos, false,
                          Scalar(this->linesToDraw[i].col.red(),this->linesToDraw[i].col.green(),this->linesToDraw[i].col.blue()),
                          this->linesToDraw[i].thickness, LINE_8, 0 );

            if (!this->linesToDraw[i].text.empty()){
                cv::putText(display, this->linesToDraw[i].text,
                            this->linesToDraw[i].pos[0]+Point(-15,10) , //+Point(this->circsToDraw[i].r,-this->circsToDraw[i].r),
                        FONT_HERSHEY_DUPLEX, 1,
                        Scalar(this->linesToDraw[i].col.red(),this->linesToDraw[i].col.green(),this->linesToDraw[i].col.blue()), 2, 8);
            }
        }
    }

    if (!alphaCircles.empty() || !alphaLines.empty() ){
        cv::Mat overlay;
        display.copyTo(overlay);
        for (int i = 0; i < alphaCircles.size(); ++i) {
            cv::circle(overlay,alphaCircles[i].pos, alphaCircles[i].r,
                       Scalar(alphaCircles[i].col.red(),alphaCircles[i].col.green(),alphaCircles[i].col.blue()),
                       alphaCircles[i].thickness);

            if (!alphaCircles[i].text.empty()){
                cv::putText(overlay, alphaCircles[i].text,
                            alphaCircles[i].pos+Point(-15,10) , //+Point(this->circsToDraw[i].r,-this->circsToDraw[i].r),
                            FONT_HERSHEY_DUPLEX, 1,
                            Scalar(alphaCircles[i].col.red(),alphaCircles[i].col.green(),alphaCircles[i].col.blue()), 2, 8);
            }
        }
        for (int i = 0; i < alphaLines.size(); ++i) {
            cv::polylines(display, alphaLines[i].pos, false,
                          Scalar(alphaLines[i].col.red(),alphaLines[i].col.green(),alphaLines[i].col.blue()),
                          alphaLines[i].thickness, LINE_8, 0 );

            if (!alphaLines[i].text.empty()){
                cv::putText(display, alphaLines[i].text,
                            alphaLines[i].pos[0]+Point(-15,10) , //+Point(this->circsToDraw[i].r,-this->circsToDraw[i].r),
                        FONT_HERSHEY_DUPLEX, 1,
                        Scalar(alphaLines[i].col.red(),alphaLines[i].col.green(),alphaLines[i].col.blue()), 2, 8);
            }
        }
        double alpha = 0.2;
        cv::addWeighted(display, 1.0-alpha, overlay, alpha , 0.0, display);
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
    bb.width = bb.x + bb.width < this->finalImageB.size().width ? bb.width :  this->finalImageB.size().width - bb.x - 1;
    bb.y = bb.y > 0 ? bb.y : 0;
    bb.height = bb.y + bb.height < this->finalImageB.size().height ? bb.height :  this->finalImageB.size().height - bb.y - 1;

    return bb;

}

// Color detection V1: green detection works only at low intensity of the ambiant light
#ifdef COLDET_V1
void KilobotTracker::getKiloBotLights(Mat &display) {
    // use CUDA to find the kilobot lights...

    // set up three streams to try to get concurrent kernels
    cuda::Stream stream1;
    cuda::Stream stream2;
    cuda::Stream stream3;

    // calculate differences
    cuda::GpuMat channelRlow;

    cuda::GpuMat channelGlow;
    cuda::GpuMat channelGhigh;

    cuda::GpuMat channelBlow;

    cuda::multiply(finalImageR,0.6,channelRlow,1,-1,stream2);
    cuda::multiply(finalImageG,0.56,channelGlow,1,-1,stream3);
    cuda::multiply(finalImageB,0.65,channelBlow,1,-1,stream1);



    //cuda::multiply(finalImageR,0.55,channelRhigh,1,-1,stream2);
    //cuda::multiply(finalImageG,0.70,channelGhigh,1,-1,stream3);
    cuda::multiply(finalImageG,0.75,channelGhigh,1,-1,stream3);
    //cuda::multiply(finalImage, 0.55,channelBhigh,1,-1,stream1);

    cuda::GpuMat bg;
    cuda::add(channelBlow,channelGhigh,bg, cuda::GpuMat(),-1,stream1);

    cuda::GpuMat rg;
    cuda::add(channelRlow,channelGlow,rg, cuda::GpuMat(),-1,stream2);

    cuda::GpuMat br;
    cuda::add(channelBlow,channelRlow,br, cuda::GpuMat(),-1,stream3);

    cuda::GpuMat b;
    cuda::GpuMat g;
    cuda::GpuMat r;
    cuda::subtract(finalImageR,bg,r,cuda::GpuMat(),-1,stream1);
    cuda::subtract(finalImageG,br,g,cuda::GpuMat(),-1,stream3);
    cuda::subtract(finalImageB,rg,b,cuda::GpuMat(),-1,stream2);

    // **********************************************************************
    // *********************** FOR DEBUG ************************************
    //    for (size_t k = 0; k < kilos.size(); ++k) {
    //        Point center(kilos[k]->getPosition().x(),kilos[k]->getPosition().y());
    //        circle( display, center, float(this->kbMaxSize)/1.4, Scalar(250,250,50), 1, 8, 0 );
    //    }
    // **********************************************************************
#ifdef TESTLEDS
    cuda::GpuMat yay;
    cuda::multiply(b,3.0,yay,1,-1,stream2);
    yay.download(display);
    cv::cvtColor(display,display,CV_GRAY2RGB);
#endif

    int circlyness = 6;

    QVector < bool > isBlue;
    isBlue.resize(this->kilos.size());

    QVector < bool > updated;
    updated.resize(this->kilos.size());
    // BLUE
    if(m_detectblue){


        int circle_acc = circlyness;
        cuda::GpuMat circlesGpu;

        vector < cuda::GpuMat > circChans;
        vector < cuda::GpuMat > kbChans;

        this->hough2->setVotesThreshold(circle_acc);
        this->hough2->detect(b,circlesGpu,stream1);

        // get the channels so we can get rid of the sizes and use locations only
        cuda::split(circlesGpu,circChans,stream1);
        cuda::split(kbLocs,kbChans,stream1);

        // create ones
        cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
        cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

        // expanded mats
        cuda::GpuMat all_x_c;
        cuda::GpuMat all_y_c;
        cuda::GpuMat all_x_kb;
        cuda::GpuMat all_y_kb;

#ifdef TESTLEDS
        Mat xCpu;
        Mat yCpu;
        circChans[0].download(xCpu);
        circChans[1].download(yCpu);

        for (int i = 0; i < xCpu.size().width;++i) {
            cv::circle(display,Point(mean(xCpu(Rect(i,0,1,1)))[0],mean(yCpu(Rect(i,0,1,1)))[0]),3, Scalar(250,250,50), 1, 8, 0);
        }
#endif

        if (circChans[0].size().width  > 0) {


            // expand circle x's & y's
            vector < cuda::Stream > streams(4);
            cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
            cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

            // expand kb x's & y's
            cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
            cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

            // diffs
            cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
            cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

            // distances
            cuda::magnitude(all_x_c,all_y_c,all_x_c);

            double * min = new double;
            Point * minLoc = new Point();

            Mat localDists;

            all_x_c.download(localDists);

            //cout << endl << localDists << endl;

            // download circChans
            Mat circChansXCpu;
            Mat circChansYCpu;

            circChans[0].download(circChansXCpu);
            circChans[1].download(circChansYCpu);

            // min
            for (int i = 0; i < this->kilos.size(); ++i) {
                minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                //                qDebug() << "robot" << kilos[i]->getID();
                //                qDebug() << "[" << float(this->kbMaxSize)/2.0 << "] minDist is" << *min;
                // work out if we should update...
                if (*min < float(this->kbMaxSize)) {

                    lightColour col;
                    col = BLUE;
                    isBlue[i] = true;
                    updated[i] = true;

                    // on the cpu
                    //                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), col);
                    kilos[i]->colBuffer.addColour(col);
                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
                }
            }

        }
    }
    // RED
    if(m_detectred){

        int circle_acc = circlyness;
        cuda::GpuMat circlesGpu;

        vector < cuda::GpuMat > circChans;
        vector < cuda::GpuMat > kbChans;

        this->hough2->setVotesThreshold(circle_acc);
        this->hough2->detect(r,circlesGpu,stream1);

        // get the channels so we can get rid of the sizes and use locations only
        cuda::split(circlesGpu,circChans,stream1);
        cuda::split(kbLocs,kbChans,stream1);

        // create ones
        cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
        cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

        // expanded mats
        cuda::GpuMat all_x_c;
        cuda::GpuMat all_y_c;
        cuda::GpuMat all_x_kb;
        cuda::GpuMat all_y_kb;

        //qDebug() << circChans[0].size().width;

        if (circChans[0].size().width  > 0) {


            // expand circle x's & y's
            vector < cuda::Stream > streams(4);
            cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
            cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

            // expand kb x's & y's
            cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
            cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

            // diffs
            cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
            cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

            // distances
            cuda::magnitude(all_x_c,all_y_c,all_x_c);

            double * min = new double;
            Point * minLoc = new Point();

            Mat localDists;

            all_x_c.download(localDists);

            //cout << endl << localDists << endl;

            // download circChans
            Mat circChansXCpu;
            Mat circChansYCpu;

            circChans[0].download(circChansXCpu);
            circChans[1].download(circChansYCpu);

            // min
            for (int i = 0; i < this->kilos.size(); ++i) {
                minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                //                qDebug() << "robot" << kilos[i]->getID();
                //                qDebug() << "[" << float(this->kbMaxSize)/2.0 << "] minDist is" << *min;
                // work out if we should update...
                if (*min < float(this->kbMaxSize)) {

                    lightColour col;
                    col = RED;
                    updated[i] = true;

                    //                    if (isBlue[i]){
                    //                        qDebug() << "  * * * * * WE GOT A POSSIBLE DETECTION ERROR! (on robot " << kilos[i]->getID() << ") * * * * *";
                    //                    }

                    // on the cpu
                    //                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), col);
                    kilos[i]->colBuffer.addColour(col);
                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
                }
            }

        }
    }
    // Green
    if(m_detectgreen){
        int circle_acc = circlyness;
        cuda::GpuMat circlesGpu;

        vector < cuda::GpuMat > circChans;
        vector < cuda::GpuMat > kbChans;

        this->hough2->setVotesThreshold(circle_acc);
        this->hough2->detect(g,circlesGpu,stream1);

        // get the channels so we can get rid of the sizes and use locations only
        cuda::split(circlesGpu,circChans,stream1);
        cuda::split(kbLocs,kbChans,stream1);

        // create ones
        cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
        cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

        // expanded mats
        cuda::GpuMat all_x_c;
        cuda::GpuMat all_y_c;
        cuda::GpuMat all_x_kb;
        cuda::GpuMat all_y_kb;

        //qDebug() << circChans[0].size().width;

        if (circChans[0].size().width  > 0) {


            // expand circle x's & y's
            vector < cuda::Stream > streams(4);
            cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
            cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

            // expand kb x's & y's
            cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
            cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

            // diffs
            cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
            cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

            // distances
            cuda::magnitude(all_x_c,all_y_c,all_x_c);

            double * min = new double;
            Point * minLoc = new Point();

            Mat localDists;

            all_x_c.download(localDists);

            //cout << endl << localDists << endl;

            // download circChans
            Mat circChansXCpu;
            Mat circChansYCpu;

            circChans[0].download(circChansXCpu);
            circChans[1].download(circChansYCpu);

            // min
            for (int i = 0; i < this->kilos.size(); ++i) {
                minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                //                qDebug() << "robot" << kilos[i]->getID();
                //                qDebug() << "[" << float(this->kbMaxSize)/2.0 << "] minDist is" << *min;
                // work out if we should update...
                if (*min < float(this->kbMaxSize)) {

                    lightColour col;
                    col = GREEN;
                    updated[i] = true;

                    //                    if (isBlue[i]){
                    //                        qDebug() << "  * * * * * WE GOT A POSSIBLE DETECTION ERROR! (on robot " << kilos[i]->getID() << ") * * * * *";
                    //                    }

                    // on the cpu
                    //                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), col);
                    kilos[i]->colBuffer.addColour(col);
                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());

                }
            }

        }
    }
    // Set the robots which have not been updated as LED=off
    for (int i = 0; i < this->kilos.size(); ++i) {

        if (!updated[i]){
            kilos[i]->colBuffer.addColour(OFF);
            kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
        }
    }
}
#endif

// Color detection V2: green detection works for both high and low intensity of the ambiant light but the frame rate is slightly lower than V1
#ifdef COLDET_V2
void KilobotTracker::getKiloBotLights(Mat &display) {
    // use CUDA to find the kilobot lights...

    // set up three streams to try to get concurrent kernels
    cuda::Stream stream1;
    cuda::Stream stream2;
    cuda::Stream stream3;


    cuda::GpuMat colorimageRGB,colorimageLAB;
    cuda::merge(vector < cuda::GpuMat > ({finalImageR,finalImageG,finalImageB}),colorimageRGB,stream1);
    cuda::cvtColor(colorimageRGB,colorimageLAB,CV_BGR2Lab,3,stream2);
    vector < cuda::GpuMat > colorimagechans(3);
    cuda::split(colorimageLAB,colorimagechans,stream3);

    clahe=cuda::createCLAHE(10);
    cuda::GpuMat dst;

    clahe->apply(colorimagechans[0],dst,stream1);
    dst.copyTo(colorimagechans[0]);

    clahe->apply(colorimagechans[1],dst,stream2);
    dst.copyTo(colorimagechans[1]);

    clahe=cuda::createCLAHE(5);
    clahe->apply(colorimagechans[2],dst,stream3);
    dst.copyTo(colorimagechans[2]);


    cuda::merge(colorimagechans,colorimageLAB,stream1);
    cuda::cvtColor(colorimageLAB,colorimageRGB,CV_Lab2BGR,3,stream2);
    cuda::split(colorimageRGB,colorimagechans,stream3);


    cuda::GpuMat channelRlow;
    cuda::GpuMat channelGlow;
    cuda::GpuMat channelBlow;
    cuda::GpuMat channelGhigh;

    cuda::GpuMat channelR_G;
    cuda::GpuMat channelB_G;

    cuda::multiply(finalImageR,0.6,channelRlow,1,-1,stream2);
    cuda::multiply(finalImageG,0.56,channelGlow,1,-1,stream3);
    cuda::multiply(finalImageB,0.65,channelBlow,1,-1,stream1);
    cuda::multiply(finalImageG,0.75,channelGhigh,1,-1,stream3);

    cuda::add(finalImageR,colorimagechans[0],colorimagechans[0], cuda::GpuMat(),-1,stream1);
    cuda::add(finalImageG,colorimagechans[1],colorimagechans[1], cuda::GpuMat(),-1,stream1);
    // finalImageB instead of finalImageG
    cuda::add(finalImageG,colorimagechans[2],colorimagechans[2], cuda::GpuMat(),-1,stream1);

    cuda::multiply(colorimagechans[0],0.7,channelR_G,1,-1,stream1);
    cuda::multiply(colorimagechans[2],0.6,channelB_G,1,-1,stream3);

    cuda::GpuMat bg;
    cuda::add(channelBlow,channelGhigh,bg, cuda::GpuMat(),-1,stream1);

    cuda::GpuMat rg;
    cuda::add(channelRlow,channelGlow,rg, cuda::GpuMat(),-1,stream2);

    cuda::GpuMat br;
    cuda::add(channelB_G,channelR_G,br, cuda::GpuMat(),-1,stream3);

    cuda::GpuMat b;
    cuda::GpuMat g;
    cuda::GpuMat r;

    cuda::subtract(finalImageR,bg,r,cuda::GpuMat(),-1,stream1);
    cuda::subtract(colorimagechans[1],br,g,cuda::GpuMat(),-1,stream3);
    cuda::threshold(g,g,15,255,CV_THRESH_BINARY,stream3);
    cuda::subtract(finalImageB,rg,b,cuda::GpuMat(),-1,stream2);



#ifdef TESTLEDS
    //    cuda::GpuMat yay;
    //    cuda::multiply(fullthresh,3.0,yay,1,-1,stream2);
    //    mask.download(display);
    g.download(display);
    cv::cvtColor(display,display,CV_GRAY2RGB);
#endif


    int circlyness = 7;
    QVector < bool > updated;
    updated.resize(this->kilos.size());

    if(m_detectblue){
        // BLUE

        int circle_acc = circlyness;
        cuda::GpuMat circlesGpu;

        vector < cuda::GpuMat > circChans;
        vector < cuda::GpuMat > kbChans;

        this->hough2->setVotesThreshold(circle_acc);
        this->hough2->detect(b,circlesGpu,stream1);

        // get the channels so we can get rid of the sizes and use locations only
        cuda::split(circlesGpu,circChans,stream1);
        cuda::split(kbLocs,kbChans,stream1);

        // create ones
        cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
        cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

        // expanded mats
        cuda::GpuMat all_x_c;
        cuda::GpuMat all_y_c;
        cuda::GpuMat all_x_kb;
        cuda::GpuMat all_y_kb;

        //#ifdef TESTLEDS
        //        Mat xCpu;
        //        Mat yCpu;
        //        circChans[0].download(xCpu);
        //        circChans[1].download(yCpu);

        //        for (int i = 0; i < xCpu.size().width;++i) {
        //            cv::circle(display,Point(mean(xCpu(Rect(i,0,1,1)))[0],mean(yCpu(Rect(i,0,1,1)))[0]),3,Scalar(255,0,0),2);
        //        }
        //#endif

        if (circChans[0].size().width  > 0) {


            // expand circle x's & y's
            vector < cuda::Stream > streams(4);
            cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
            cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

            // expand kb x's & y's
            cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
            cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

            // diffs
            cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
            cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

            // distances
            cuda::magnitude(all_x_c,all_y_c,all_x_c);

            double * min = new double;
            Point * minLoc = new Point();

            Mat localDists;

            all_x_c.download(localDists);

            //cout << endl << localDists << endl;

            // download circChans
            Mat circChansXCpu;
            Mat circChansYCpu;

            circChans[0].download(circChansXCpu);
            circChans[1].download(circChansYCpu);

            for (int i = 0; i < this->kilos.size(); ++i) {

                if (!updated[i]){
                    minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                    //                qDebug() << "robot" << kilos[i]->getID();
                    //                qDebug() << "[" << float(this->kbMaxSize)/2.0 << "] minDist is" << *min;
                    // work out if we should update...
                    if (*min < float(this->kbMaxSize)) {

                        lightColour col;
                        col = BLUE;
                        updated[i] = true;

                        // on the cpu
                        //                    kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), col);
                        kilos[i]->colBuffer.addColour(col);
                        kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
                    }
                }
            }

        }
    }

    if(m_detectred){
        // RED

        int circle_acc = circlyness;
        cuda::GpuMat circlesGpu;

        vector < cuda::GpuMat > circChans;
        vector < cuda::GpuMat > kbChans;

        this->hough2->setVotesThreshold(circle_acc);
        this->hough2->detect(r,circlesGpu,stream1);

        //        // FOR DEGUB: draw all the circles found through GPU-hough
        //        vector<Vec3f> circles;
        //        circlesGpu.download(circles);
        //        for( size_t i = 0; i < circles.size(); i++ )
        //        {
        //            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        //            int radius = cvRound(circles[i][2]);
        //            // draw the circle outline
        //            circle( display, center, radius, Scalar(250,250,50), 1, 8, 0 );
        //        }

        // get the channels so we can get rid of the sizes and use locations only
        cuda::split(circlesGpu,circChans,stream1);
        cuda::split(kbLocs,kbChans,stream1);

        // create ones
        cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
        cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

        // expanded mats
        cuda::GpuMat all_x_c;
        cuda::GpuMat all_y_c;
        cuda::GpuMat all_x_kb;
        cuda::GpuMat all_y_kb;

        //qDebug() << circChans[0].size().width;

        if (circChans[0].size().width  > 0) {


            // expand circle x's & y's
            vector < cuda::Stream > streams(4);
            cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
            cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

            // expand kb x's & y's
            cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
            cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

            // diffs
            cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
            cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

            // distances
            cuda::magnitude(all_x_c,all_y_c,all_x_c);

            double * min = new double;
            Point * minLoc = new Point();

            Mat localDists;

            all_x_c.download(localDists);

            //cout << endl << localDists << endl;

            // download circChans
            Mat circChansXCpu;
            Mat circChansYCpu;

            circChans[0].download(circChansXCpu);
            circChans[1].download(circChansYCpu);

            // min
            for (int i = 0; i < this->kilos.size(); ++i) {

                if (!updated[i]){
                    minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);

                    if (*min < float(this->kbMaxSize)) {

                        lightColour col;
                        col = RED;
                        updated[i] = true;
                        kilos[i]->colBuffer.addColour(col);
                        kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
                    }
                }
            }

        }
    }

    if(m_detectgreen){
        // Green
        int circle_acc = circlyness;
        cuda::GpuMat circlesGpu;

        vector < cuda::GpuMat > circChans;
        vector < cuda::GpuMat > kbChans;

        this->hough2->setVotesThreshold(circle_acc);
        this->hough2->detect(g,circlesGpu,stream1);

        // get the channels so we can get rid of the sizes and use locations only
        cuda::split(circlesGpu,circChans,stream1);
        cuda::split(kbLocs,kbChans,stream1);

        // create ones
        cuda::GpuMat ones_kb(kbChans[0].size(),kbChans[0].type(),1);
        cuda::GpuMat ones_c(circChans[0].size(),circChans[0].type(),1);

        // expanded mats
        cuda::GpuMat all_x_c;
        cuda::GpuMat all_y_c;
        cuda::GpuMat all_x_kb;
        cuda::GpuMat all_y_kb;

        //qDebug() << circChans[0].size().width;

        if (circChans[0].size().width  > 0) {


            // expand circle x's & y's
            vector < cuda::Stream > streams(4);
            cuda::gemm(circChans[0], ones_kb, 1.0, noArray(), 0.0, all_x_c,GEMM_1_T,streams[0]);
            cuda::gemm(circChans[1], ones_kb, 1.0, noArray(), 0.0, all_y_c,GEMM_1_T,streams[1]);

            // expand kb x's & y's
            cuda::gemm(ones_c, kbChans[0], 1.0, noArray(), 0.0, all_x_kb,GEMM_1_T,streams[2]);
            cuda::gemm(ones_c, kbChans[1], 1.0, noArray(), 0.0, all_y_kb,GEMM_1_T,streams[3]);

            // diffs
            cuda::subtract(all_x_c,all_x_kb,all_x_c,noArray(),-1,streams[0]);
            cuda::subtract(all_y_c,all_y_kb,all_y_c,noArray(),-1,streams[1]);

            // distances
            cuda::magnitude(all_x_c,all_y_c,all_x_c);

            double * min = new double;
            Point * minLoc = new Point();

            Mat localDists;

            all_x_c.download(localDists);

            //cout << endl << localDists << endl;

            // download circChans
            Mat circChansXCpu;
            Mat circChansYCpu;

            circChans[0].download(circChansXCpu);
            circChans[1].download(circChansYCpu);

            // min
            for (int i = 0; i < this->kilos.size(); ++i) {

                if (!updated[i]){
                    minMaxLoc(localDists(Rect(i,0,1,localDists.size().height)),min,NULL,minLoc,NULL);
                    if (*min < float(this->kbMaxSize)/1.8) {

                        lightColour col;

                        col = GREEN;
                        updated[i] = true;
                        kilos[i]->colBuffer.addColour(col);
                        kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
                    }

                }
            }

        }
    }


    for (int i = 0; i < this->kilos.size(); ++i) {

        if (!updated[i]){
            kilos[i]->colBuffer.addColour(OFF);
            kilos[i]->updateState(kilos[i]->getPosition(),kilos[i]->getVelocity(), kilos[i]->colBuffer.getAvgColour());
        }
    }

}
#endif


#ifdef USE_CUDA
kiloLight KilobotTracker::getKiloBotLight(cuda::GpuMat channelsG[3], Point centreOfBox, int index)
#else
kiloLight KilobotTracker::getKiloBotLight(Mat channels[3], Point centreOfBox, int index)
#endif
{
    // find the location and colour of the light...

    kiloLight light;
    light.pos = Point(-1,-1);
#ifndef USE_CUDA


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
#endif
    return light;

}

#ifdef USE_CUDA
kiloLight KilobotTracker::getKiloBotLightAdaptive(cuda::GpuMat channels[3], Point centreOfBox, int index)
#else
kiloLight KilobotTracker::getKiloBotLightAdaptive(Mat channels[3], Point centreOfBox, int index)
#endif
{
    // find the location and colour of the light...

    kiloLight light;
    light.pos = Point(-1,-1);

#ifndef USE_CUDA


    vector < Mat > temp(3);
    Scalar sums[3];

    float tooBig = 2000.0f;//10000.0f
    int step = 5;

    uint maxIndex = 0;

    // find colour
    for (uint i = 0; i < 3; ++i) {
        CV_NS threshold(channels[i], temp[i], kilos[index]->lightThreshold,255,CV_THRESH_TOZERO);
        temp[i] = temp[i] - kilos[index]->lightThreshold;
        sums[i] = CV_NS sum(temp[i]);
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
#endif
    return light;

}

void KilobotTracker::SETUPloadCalibration()
{
    if( (this->srcType==VIDEO) && (this->videoPath.isEmpty()))
        emit errorMessage("No video file selected!");
    else
    {
        // Load the calibration data

        // nicety - load last used directory
        QSettings settings;
        QString lastDir = settings.value("lastDirOut", QDir::homePath()).toString();
        QString fileName = QFileDialog::getOpenFileName((QWidget *) sender(), tr("Load Calibration"), lastDir, tr("Calib files (*.xml *.yaml);; All files (*)"));

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
        //this->corners[i].x = max(-1535, this->corners[i].x); //TEMP
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

    fullCorner =  Point(min_x, min_y);
    fullSize =  Size(max_x-min_x+1, max_y-min_y+1);

    // assign indices...
    for (int j = 0; j < 4; ++j) {
        if (corners[j].x - fullCorner.x < fullSize.width/5 && corners[j].y - fullCorner.y < fullSize.height/5) {
            clData.inds[0] = j;
        } else if (corners[j].x - fullCorner.x > fullSize.width/5 && corners[j].y - fullCorner.y < fullSize.height/5) {
            clData.inds[1] = j;
        } else if (corners[j].x - fullCorner.x < fullSize.width/5 && corners[j].y - fullCorner.y > fullSize.height/5) {
            clData.inds[2] = j;
        } else if (corners[j].x - fullCorner.x > fullSize.width/5 && corners[j].y - fullCorner.y > fullSize.height/5) {
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
        this->threads[i]->height_x_adj = this->height_x_adj;
        this->threads[i]->height_y_adj = this->height_y_adj;
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
    //if (this->flip180) cv::flip(display, display,-1);

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
            for (int i = 0; i < 4; ++i)
            {
                camOrder[i] = temp[i];
                /*if (this->threads[i]) {
                    this->threads[i]->index =
                }*/
            }
            qDebug() << "Cam order set";
        }

    }

}

void KilobotTracker::RefreshDisplayedImage()
{
    if(this->threads[0] && !this->threads[0]->isRunning()){

        if(this->haveCalibration){
            // launch threads
            for (int i=0;i<2;i++){
                this->THREADSlaunch();

                this->time =0;
                // run stitcher once
                this->loadFirstIm = true;
                this->trackType=NO_TRACK;
                this->LOOPiterate();
                this->trackType=CIRCLES_NAIVE;
                this->loadFirstIm = false;
                this->THREADSstop();

                this->time = 0;
            }
        }
        else emit errorMessage(QString("No arena calibration loaded yet!"));
    }
    else emit errorMessage(QString("Your can't rotate the display when experiment is running!"));

}


void KilobotTracker::setFlipangle(double angle) {
    if(this->threads[0] && !this->threads[0]->isRunning() ){
        flipangle=flipangle+angle;
        if( (flipangle==360) || (flipangle==-360) ) flipangle=0;
    }
}

/* method to move the position of kilobot (with known ID) to specific position inicated by the user through a mouse click */
void KilobotTracker::manuallyassignID(QPoint position){

    if(m_assignIDmanually){
        position *= 2000.0/600.0;
        Mat tempKbLocs(1,kilos.size(), CV_32FC2);
        float * data = (float *) tempKbLocs.data;
        for (int i = 0; i < kilos.size(); ++i) {
            if(kilos[i]->getID()==manualID){
                kilos[i]->updateState(position,kilos[i]->getVelocity(),kilos[i]->getLedColour());
                qDebug() << "robot: " << manualID << "has been repositioned" ;
            }

            data[i*2] = kilos[i]->getPosition().x();
            data[i*2+1] = kilos[i]->getPosition().y();
        }
        kbLocs.upload(tempKbLocs);
    }
}

