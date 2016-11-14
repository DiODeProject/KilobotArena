#include "mainwindow.h"
#include "ui_mainwindow.h"

// QT includes
#include <QLabel>
#include <QLayout>
#include <QDebug>
#include <QSettings>
#include <QDir>
#include <QFileDialog>
#include <QSignalMapper>

// STL includes
#include <vector>

// this enables us to 'talk' kilobot
#include "ohc/packet.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(&this->kbtracker,SIGNAL(errorMessage(QString)), ui->error_label, SLOT(setText(QString)));
    connect(&this->ohc,SIGNAL(errorMessage(QString)), ui->error_label, SLOT(setText(QString)));

    connect(&this->kbtracker,SIGNAL(identifyKilo(kilobot_id)), &this->ohc, SLOT(identifyKilobot(kilobot_id)));
    connect(&this->kbtracker,SIGNAL(broadcastMessage(kilobot_message_type,kilobot_message_data)), &this->ohc, SLOT(broadcastMessage(kilobot_message_type,kilobot_message_data)));

    connect(&this->kbtracker, SIGNAL(setStitchedImage(QPixmap)),ui->result_final,SLOT(setPixmap(QPixmap)));

    connect(ui->load_calib, SIGNAL(clicked(bool)), &this->kbtracker, SLOT(loadCalibration()));


    QSignalMapper *mapper = new QSignalMapper(this);
    mapper->setMapping(ui->run, TRACK);
    mapper->setMapping(ui->identify, IDENTIFY);
    mapper->setMapping(ui->assign, ASSIGN);
    connect(ui->run, SIGNAL(clicked(bool)), mapper, SLOT(map()));
    connect(ui->identify, SIGNAL(clicked(bool)), mapper, SLOT(map()));
    connect(ui->assign, SIGNAL(clicked(bool)), mapper, SLOT(map()));
    connect(mapper, SIGNAL(mapped(int)), &this->kbtracker, SLOT(startLoop(int)));


    connect(ui->find_kb, SIGNAL(clicked(bool)), &this->kbtracker, SLOT(findKilobots()));

    connect(ui->lineEdit, SIGNAL(editingFinished()), &this->kbtracker, SLOT(setCamOrder()));

    connect(ui->cam_radio, SIGNAL(toggled(bool)), &this->kbtracker, SLOT(setSourceType(bool)));

    connect(ui->sel_video, SIGNAL(clicked(bool)), this, SLOT(setVideoSource()));

    connect(ui->houghAcc_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setHoughAcc(int)));
    connect(ui->cannyThresh_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setCannyThresh(int)));
    connect(ui->kbMax_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setKbMax(int)));
    connect(ui->kbMin_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setKbMin(int)));

    connect(ui->ohc_connect, SIGNAL(clicked(bool)), &this->ohc, SLOT(toggleConnection()));
    connect(ui->ohc_reset, SIGNAL(toggled(bool)), &this->ohc, SLOT(resetKilobots()));
    connect(ui->ohc_sleep, SIGNAL(toggled(bool)), &this->ohc, SLOT(sleepKilobots()));
    connect(ui->ohc_run, SIGNAL(toggled(bool)), &this->ohc, SLOT(runKilobots()));
    connect(ui->ohc_stop, SIGNAL(toggled(bool)), &this->ohc, SLOT(stopSending()));
    connect(ui->ohc_set_prog, SIGNAL(clicked(bool)), &this->ohc, SLOT(chooseProgramFile()));
    connect(ui->ohc_upload_prog, SIGNAL(clicked(bool)), &this->ohc, SLOT(uploadProgram()));

    connect(ui->left, SIGNAL(clicked(bool)), this, SLOT(left()));
    connect(ui->right, SIGNAL(clicked(bool)), this, SLOT(right()));
    connect(ui->straight, SIGNAL(clicked(bool)), this, SLOT(straight()));

    connect(&this->ohc,SIGNAL(setStopButton(bool)),ui->ohc_stop,SLOT(setChecked(bool)));

    connect(ui->test, SIGNAL(clicked(bool)), this, SLOT(test_id()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setVideoSource()
{

    QSettings settings;
    QString lastDir = settings.value("videoLastDir", QDir::homePath()).toString();
    QString dirName = QFileDialog::getExistingDirectory(this, tr("Set the video source"), lastDir);

    if (dirName.isEmpty()) {
        ui->error_label->setText("No path selected");
    }

    // set dir
    this->kbtracker.setVideoDir(dirName);
    ui->vid_path->setText(dirName);
    ui->vid_path->setToolTip(dirName);
    settings.setValue ("videoLastDir", dirName);
}

void MainWindow::left()
{
    //ohc.signalKilobot(0,2,0);
    ohc.broadcastMessage(3,0);
}

void MainWindow::right()
{
    ohc.signalKilobot(0,3,0);
}

void MainWindow::straight()
{
    ohc.signalKilobot(0,1,0);
}

void MainWindow::test_id()
{
    ohc.identifyKilobot(ui->test_id->text().toInt());
}
