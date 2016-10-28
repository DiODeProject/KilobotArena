#include "mainwindow.h"
#include "ui_mainwindow.h"

// QT includes
#include <QLabel>
#include <QLayout>
#include <QDebug>
#include <QSettings>
#include <QDir>
#include <QFileDialog>

// STL includes
#include <vector>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(&this->kbtracker,SIGNAL(errorMessage(QString)), ui->error_label, SLOT(setText(QString)));

    connect(&kbtracker, SIGNAL(setStitchedImage(QPixmap)),ui->result_final,SLOT(setPixmap(QPixmap)));

    connect(ui->load_calib, SIGNAL(clicked(bool)), &this->kbtracker, SLOT(loadCalibration()));

    connect(ui->run, SIGNAL(clicked(bool)), &this->kbtracker, SLOT(startLoop()));

    connect(ui->find_kb, SIGNAL(clicked(bool)), &this->kbtracker, SLOT(findKilobots()));

    connect(ui->lineEdit, SIGNAL(editingFinished()), &this->kbtracker, SLOT(setCamOrder()));

    connect(ui->houghAcc_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setHoughAcc(int)));
    connect(ui->cannyThresh_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setCannyThresh(int)));
    connect(ui->kbMax_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setKbMax(int)));
    connect(ui->kbMin_slider, SIGNAL(valueChanged(int)), &this->kbtracker, SLOT(setKbMin(int)));


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadImages()
{
/*
    QSettings settings;
    QString lastDir = settings.value("lastDir", QDir::homePath()).toString();
    QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Load the Four Calibration Images"), lastDir, tr("Image files (*.jpg *.png);; All files (*)"));

    if (fileNames.size() != 4) {
        ui->error_label->setText("Four calibration images are required");
        return;
    }

    vector <Mat> imgs;

    for (uint i = 0; i < fileNames.size(); ++i) {
        imgs.push_back(imread(fileNames[i].toStdString(), CV_LOAD_IMAGE_COLOR));
        if (!imgs.back().data) {
            ui->error_label->setText("Error loading an image");
            return;
        }
    }

    //calibrater.setCalibrationImages(imgs);

    QDir lastDirectory (fileNames[0]);
    lastDirectory.cdUp();
    settings.setValue ("lastDir", lastDirectory.absolutePath());*/
}


