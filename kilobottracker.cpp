#include "kilobottracker.h"
#include <QImage>
#include <QDebug>
#include <QThread>
#include <QLineEdit>
#include <QDir>
#include <QSettings>
#include <QFileDialog>


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

    // reprojection details
    Mat K;
    Mat R;

    // the index of the source
    uint index = 0;

    bool keepRunning = true;

    srcDataType type = CAMERA;


private:
    /*!
     * \brief run
     * The execution method for the thread, performing the stitching process
     */
    void run() {

        int time = 0;
        Mat image;
        Mat mask;

        cv::VideoCapture cap;

        Ptr<WarperCreator> warper_creator;
        warper_creator = makePtr<cv::PlaneWarper>();
        Ptr<detail::RotationWarper> warper = warper_creator->create(2000.0f);

        // loop
        while (keepRunning) {

            // check for stop signal
            if (srcStop[index].available()) {
                keepRunning = false;
            }

            if (srcFree[index].available()) {

                // get data
                if (type == IMAGES) {
                    image = imread((QString("/home/alex/Dropbox/temp/test-with-alex/frame_00200_")+QString::number(index)+QString(".jpg")).toStdString());
                }
                else if (type == CAMERA) {
                    camUsage.acquire();
                    if (!cap.isOpened()) {
                        cap.open(camOrder[index]);
                        //QThread::sleep(1);
                        // set REZ
                        if (cap.isOpened()) {
                            cap.set(CV_CAP_PROP_FRAME_WIDTH, 2048);
                            cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1536);
                        } else {
                            this->keepRunning = false;
                            continue;
                        }
                        //QThread::sleep(1);
                    }
                    if (cap.isOpened()) cap >> image;
                    camUsage.release();
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

                srcUsed[index].release();

                ++time;
            }

        }

        if (type == CAMERA) {
            // shut down cam
            camUsage.acquire();
            if (cap.isOpened()) cap.release();
            camUsage.release();
            //QThread::sleep(1);
        }
    }
};


KilobotTracker::KilobotTracker(QPoint smallImageSize, QObject *parent) : QObject(parent)
{
    this->smallImageSize = smallImageSize;
    this->tick.setInterval(1);
    connect(&this->tick, SIGNAL(timeout()), this, SLOT(stitchImages()));

    // initialise semaphores
    srcFree[0].release(5);
    srcFree[1].release(5);
    srcFree[2].release(5);
    srcFree[3].release(5);

    camUsage.release(1);

    // KCF is the fastest algorithm and it is still VERY slow...
    this->tracker = new MultiTracker("KCF");
}

KilobotTracker::~KilobotTracker()
{
    // clean up memory
    for (uint i = 0; i < 4; ++i) {
        if (this->threads[i]) {
            delete this->threads[i];
        }
    }
    if (tracker) {
        delete tracker;
    }
}

void KilobotTracker::startLoop()
{

    // check if running
    if (this->threads[0] && this->threads[0]->isRunning()) {

        emit errorMessage(QString("FPS = ") + QString::number(float(time)/(float(timer.elapsed())/1000.0f)));

        // cancel
        srcStop[0].release();
        srcStop[1].release();
        srcStop[2].release();
        srcStop[3].release();

        // wait for thread completion
        //for (uint i = 0; i < 4; ++i) this->threads[i]->wait();

        return;

    }

    // only if we have calib data
    if (!this->haveCalibration) {
        return;
    }

    // launch threads
    for (uint i = 0; i < 4; ++i)
    {
        if (srcStop[i].available()) srcStop[i].acquire();
        if (!this->threads[i]) {
            this->threads[i] = new acquireThread;
        }
        this->threads[i]->R = this->Rs[i];
        this->threads[i]->K = this->Ks[i];
        this->threads[i]->keepRunning = true;
        this->threads[i]->index = i;
        this->threads[i]->start();
    }

    if (!this->compensator) {
        // calculate to compensate for exposure
        compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);
    }

    this->warpedImages.resize(4);
    this->warpedMasks.resize(4);
    this->corners.resize(4);
    this->sizes.resize(4);

    // start timer
    this->time = 0;
    this->last_time = 0.0f;
    this->tick.start();
    this->timer.start();
}


void KilobotTracker::stitchImages()
{

    // wait for semaphores
    if ((srcUsed[0].available() > 0 && \
        srcUsed[1].available() > 0 && \
        srcUsed[2].available() > 0 && \
        srcUsed[3].available() > 0) || this->loadFirstIm)
    {

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
            compensator->apply(i, corners[i], warpedImages[i], warpedMasks[i]);
        }

        // feather the images together
        Ptr<detail::Blender> blender;
        blender = detail::Blender::createDefault(detail::Blender::FEATHER, false);
        blender->prepare(corners, sizes);
        vector <Mat> images_warped_s(warpedImages.size());
        for (int i = 0; i < 4; ++i)
        {
            warpedImages[i].convertTo(images_warped_s[i], CV_16S);
            blender->feed(images_warped_s[i], warpedMasks[i], corners[i]);
        }

        Mat result, result_mask;
        blender->blend(result, result_mask);

        // convert (not sure what this does, but is necessary apparantly)
        result.convertTo(result, (result.type() / 8) * 8);

        cv::resize(result, result,Size(1536,1536));

        Point2f outputQuad[4];
        outputQuad[0] = Point(0,0);
        outputQuad[1] = Point(2000,0);
        outputQuad[2] = Point(0,2000);
        outputQuad[3] = Point(2000,2000);

        Mat M = getPerspectiveTransform(this->arenaCorners,outputQuad);
        warpPerspective(result, this->finalImage, M, Size(2000,2000));

        srcFree[0].release();
        srcFree[1].release();
        srcFree[2].release();
        srcFree[3].release();

        //this->trackKilobots();

        //Mat display = this->finalImage;

        // threshold
        Mat display;

        //cv::cvtColor(this->finalImage, this->finalImage, CV_BGR2GRAY);
        Mat channel[3];

        cv::split(this->finalImage, channel);

        this->finalImage = channel[0];
        this->finalImageR = channel[0];
        this->finalImageG = channel[1];
        this->finalImageB = channel[2];

        //this->findKilobots();
        this->trackKilobots();

        //cv::threshold(this->finalImage, this->finalImage, 10, 255, 3);

        if (false) {

            display = this->finalImage;

            // the *2 is an assumption - should always be true...
            cv::resize(display,display,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));
            cv::cvtColor(display, display, CV_GRAY2RGB);

            // convert to C header for easier mem ptr addressing
            IplImage imageIpl = display;

            // create a QImage container pointing to the image data
            QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

            // assign to a QPixmap (may copy)
            QPixmap pix = QPixmap::fromImage(qimg);

            setStitchedImage(pix);

        }

        ++time;

        if (time % 5 == 0) {
            float c_time = float(this->timer.elapsed())/1000.0f;
            emit errorMessage(QString("FPS = ") + QString::number(5.0f/(c_time-last_time)));
            last_time = c_time;
        }

    }

   // emit errorMessage("Stitcher thread running...");

}

void KilobotTracker::findKilobots()
{
    Mat res2;
    Mat display;
    this->finalImage.copyTo(display);

    res2 = this->finalImage;

    //cv::cvtColor(this->finalImage, res2, CV_BGR2GRAY);

    vector<Vec3f> circles;
    HoughCircles(res2,circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/,cannyThresh /* Canny threshold*/,houghAcc /*cicle algorithm accuracy*/,this->kbMaxSize-1/* circle distance*/,kbMinSize/* min circle size*/,kbMaxSize/* max circle size*/);

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

    kilobot_colour col = {0,0,0};

    for( size_t i = 0; i < circles.size(); i++ ) {

        Kilobot kilo(i,circles[i][0],circles[i][1],col,0);
        this->kilos.push_back(kilo);

    }


    /*circle( result, Point(10,20), 5, Scalar(0,255,0), 2, 8, 0 );
    circle( result, Point(500,500), 5, Scalar(0,255,0), 2, 8, 0 );
    circle( result, Point(10,500), 5, Scalar(0,255,0), 2, 8, 0 );
    circle( result, Point(500,20), 5, Scalar(0,255,0), 2, 8, 0 );

    Mat planes[3];
    split(result,planes);
    Mat res3 = planes[1]-0.8*(planes[0]+planes[2]);*/
}

void KilobotTracker::trackKilobots()
{

    //if (tracker->objects.size() > 0)
    /*if (tracker->objects.size() > 0)
    {
        tracker->update(this->finalImage);
    }

    // draw them up
    for(unsigned i=0;i<tracker->objects.size();i++) {
        rectangle( this->finalImage, tracker->objects[i], Scalar( 255, 0, 0 ), 2, 1 );
    }*/

    // my tracker:

    switch (this->trackType) {
    {
    case CIRCLES_NAIVE:

            vector<Vec3f> circles;
            HoughCircles(this->finalImage,circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/,cannyThresh /* Canny threshold*/,houghAcc /*cicle algorithm accuracy*/,this->kbMaxSize-1/* circle distance*/,kbMinSize/* min circle size*/,kbMaxSize/* max circle size*/);

            qDebug() << "Found" << circles.size() << "circles";

            float maxDist = this->kbMaxSize-2.0f;

            // find kilobots
            for( size_t i = 0; i < kilos.size(); i++ )
            {
                for (uint j = 0; j < circles.size(); ++j) {

                    Point centerK(round(kilos[i].getXPosition()), round(kilos[i].getYPosition()));
                    Point centerC(cvRound(circles[j][0]), cvRound(circles[j][1]));
                    Point diff = centerK - centerC;
                    double dist = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
                    // check
                    if (dist < maxDist) {
                        kilos[i].updateState(centerC.x, centerC.y,kilos[i].getLedColour(),0);
                    }

                }
            }

            // convert for display
            Mat display;
            cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);

            for( size_t i = 0; i < kilos.size(); i++ )
            {
                 Point center(round(kilos[i].getXPosition()), round(kilos[i].getYPosition()));
                 circle( display, center, 1, Scalar(255,0,0), 3, 8, 0 );
            }

            cv::resize(display,display,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));


            // convert to C header for easier mem ptr addressing
            IplImage imageIpl = display;

            // create a QImage container pointing to the image data
            QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

            // assign to a QPixmap (may copy)
            QPixmap pix = QPixmap::fromImage(qimg);

            setStitchedImage(pix);
            break;
    }
    {
    case CIRCLES_LOCAL:

            float maxDist = 1.5f*this->kbMaxSize;

            vector < Rect > bbs;

            // try and update each kilobot
            for (uint i = 0; i < this->kilos.size(); ++i) {

                Rect bb;
                bb.x = cvRound(this->kilos[i].getXPosition() - maxDist);
                bb.y = cvRound(this->kilos[i].getYPosition() - maxDist);
                bb.width = cvRound(maxDist*2.0f);
                bb.height = cvRound(maxDist*2.0f);

                bb.x = bb.x > 0 ? bb.x : 0;
                bb.width = bb.x + bb.width < this->finalImage.size().width ? bb.width :  this->finalImage.size().width - bb.x - 1;
                bb.y = bb.y > 0 ? bb.y : 0;
                bb.height = bb.y + bb.height < this->finalImage.size().height ? bb.height :  this->finalImage.size().height - bb.y - 1;

                bbs.push_back(bb);
            }

            // exclusion testing:

           /* if (this->exclusionTestsIndices.size() != this->kilos.size()) this->exclusionTestsIndices.resize(this->kilos.size());

            if (this->time % 10 == 0) {

                for (uint i = 0; i < this->kilos.size(); ++i) {

                    this->exclusionTestsIndices[i].clear();
                    for (uint j = 0; j < kilos.size(); ++j) {
                        if ((bbs[i] & bbs[j]).area() > 0) this->exclusionTestsIndices[i].push_back(j);
                    }

                }

            }*/

            for (uint i = 0; i < this->kilos.size(); ++i) {

                Rect bb = bbs[i];

                Mat temp = this->finalImage(bb);

                bool no_match = true;
                int circle_acc = 100;

                vector<Vec3f> circles;

                while (no_match && circle_acc > 10) {

                    // try fixed pars
                    int minS = 14;
                    int maxS = 19;

                    HoughCircles(temp,circles,CV_HOUGH_GRADIENT,1.0/* rez scaling (1 = full rez, 2 = half etc)*/,cannyThresh /* Canny threshold*/,circle_acc /*cicle algorithm accuracy*/,maxS-1/* circle distance*/,minS/* min circle size*/,maxS/* max circle size*/);

                    if (circles.size() > 0) no_match = false;

                    circle_acc -= 20;

                }

                // filter circles for exclusion
                /*for (uint k = 1; k < circles.size(); ++k) {
                    for (uint j = 0; j < this->exclusionTestsIndices[i].size(); ++j) {

                        if ((bb.x+circles[k][0] - kilos[this->exclusionTestsIndices[i][j]].getXPosition())*(bb.x+circles[k][0] - kilos[this->exclusionTestsIndices[i][j]].getXPosition()) + (bb.x+circles[k][1] - kilos[this->exclusionTestsIndices[i][j]].getYPosition())*(bb.x+circles[k][1] - kilos[this->exclusionTestsIndices[i][j]].getYPosition()) < 4*(this->kbMaxSize)*(this->kbMaxSize) ) {
                            circles.erase(circles.begin()+k);
                            k -= 1;
                        }

                    }
                }*/



                if (circles.size() > 0) {
                    // take the nearest one

                    int best_index = 0;
                    for (uint k = 1; k < circles.size(); ++k) {

                        best_index = (this->kbMaxSize-3.0)*(this->kbMaxSize-3.0) > (circles[i][0]-bb.width/2)*(circles[i][0]-bb.width/2)+(circles[i][1]-bb.height/2)*(circles[i][1]-bb.height/2) && (circles[i][0]-bb.width/2)*(circles[i][0]-bb.width/2)+(circles[i][1]-bb.height/2)*(circles[i][1]-bb.height/2) < (circles[best_index][0]-bb.width/2)*(circles[best_index][0]-bb.width/2)+(circles[best_index][1]-bb.height/2)*(circles[best_index][1]-bb.height/2) ? i : best_index;

                    }

                    int new_x = float(bb.x+circles[best_index][0])*0.5 + float(kilos[i].getXPosition())*0.5;
                    int new_y = float(bb.y+circles[best_index][1])*0.5 + float(kilos[i].getYPosition())*0.5;
                    this->kilos[i].updateState(new_x,new_y,kilos[i].getLedColour(),0);

                }

            }


            // convert for display
            Mat display;
            cv::cvtColor(this->finalImage, display, CV_GRAY2RGB);

            for( size_t i = 0; i < kilos.size(); i++ )
            {
                 Point center(round(kilos[i].getXPosition()), round(kilos[i].getYPosition()));
                 circle( display, center, 1, Scalar(255,0,0), 3, 8, 0 );
            }

            if (kilos.size() > 0) {
                Rect bb;
                bb.x = cvRound(this->kilos[0].getXPosition() - maxDist);
                bb.y = cvRound(this->kilos[0].getYPosition() - maxDist);
                bb.width = cvRound(maxDist*2.0f);
                bb.height = cvRound(maxDist*2.0f);
                rectangle(display, bb, Scalar(0,0,255),3);
            }

            cv::resize(display,display,Size(this->smallImageSize.x()*2, this->smallImageSize.y()*2));


            // convert to C header for easier mem ptr addressing
            IplImage imageIpl = display;

            // create a QImage container pointing to the image data
            QImage qimg((uchar *) imageIpl.imageData,imageIpl.width,imageIpl.height,QImage::Format_RGB888);

            // assign to a QPixmap (may copy)
            QPixmap pix = QPixmap::fromImage(qimg);

            setStitchedImage(pix);
            break;
    }
    {
    case PARTICLE_FILTER:

            break;
    }
    }



}

void KilobotTracker::loadCalibration()
{

    // Load the calibration data

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

    this->haveCalibration = true;

    QDir lastDirectory (fileName);
    lastDirectory.cdUp();
    settings.setValue ("lastDirOut", lastDirectory.absolutePath());

    // load first images

    // launch threads
    for (uint i = 0; i < 4; ++i)
    {
        if (srcStop[i].available()) srcStop[i].acquire();
        if (!this->threads[i]) {
            this->threads[i] = new acquireThread;
        }
        this->threads[i]->R = this->Rs[i];
        this->threads[i]->K = this->Ks[i];
        this->threads[i]->keepRunning = true;
        this->threads[i]->index = i;
        this->threads[i]->start();
    }

    if (!this->compensator) {
        // calculate to compensate for exposure
        compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);
    }

    this->warpedImages.resize(4);
    this->warpedMasks.resize(4);
    this->corners.resize(4);
    this->sizes.resize(4);

    this->time = 0;

    // run stitcher once
    this->loadFirstIm = true;
    this->stitchImages();
    this->loadFirstIm = false;

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
        srcFree[i].release(5);
    }
    this->time = 0;

}

void KilobotTracker::setCamOrder()
{

    QLineEdit * src = qobject_cast < QLineEdit * > (this->sender());
    if (src) {
        QString str = src->text();
        QStringList list = str.split(",");
        if (list.size() == 4)
        {
            for (int i = 0; i < list.size(); ++i) {
                camOrder[i] = list[i].toInt();
            }
        }
    }

}

void KilobotTracker::setKbMax(int val)
{
    this->kbMaxSize = val;
}

void KilobotTracker::setKbMin(int val)
{
    this->kbMinSize = val;
}

void KilobotTracker::setHoughAcc(int val)
{
    this->houghAcc = val;
}

void KilobotTracker::setCannyThresh(int val)
{
    this->cannyThresh = val;
}
