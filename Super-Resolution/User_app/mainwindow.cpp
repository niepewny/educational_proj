#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "imagehandler.h"
#include <QMessageBox>
#include <QTimer>
#include "clientinterface.h"
#include <QDir>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    centralWidget();

    preprocessedhandler = new Preprocessedhandler(ui->scrollArea, this);
    processedhandler = new Processedhandler(ui->scrollArea_2, this);

    ui->scrollArea->setBackgroundRole(QPalette::Dark);
    ui->scrollArea->setWidget(preprocessedhandler->getImageLabel());
    ui->scrollArea_2->setBackgroundRole(QPalette::Dark);
    ui->scrollArea_2->setWidget(processedhandler->getImageLabel());

    connect(ui->loadButton, &QPushButton::released, preprocessedhandler, &Preprocessedhandler::open);
    connect(ui->sendButton, &QPushButton::released, this, &MainWindow::sendImage);
    connect(ui->saveButton, &QPushButton::released, this, &MainWindow::saveImage);
    connect(ui->saveAllButton, &QPushButton::released, processedhandler, &Processedhandler::saveAll);
    connect(ui->checkButton, &QPushButton::released, processedhandler, &Processedhandler::forceCheck);
    connect(preprocessedhandler, &ImageHandler::filesReady, this, &MainWindow::updatePreTransImageList);
    connect(processedhandler, &Processedhandler::filesReady, this, &MainWindow::updateProcessedList);
    connect(ui->preTransImagesList, &QListWidget::currentRowChanged, preprocessedhandler, &Preprocessedhandler::display);
    connect(ui->processedImagesList, &QListWidget::currentRowChanged, processedhandler, &Processedhandler::display);
    connect(processedhandler, &Processedhandler::refreshRemainingTime, this, &MainWindow::refreshRemainingTime);
    connect(preprocessedhandler, &Preprocessedhandler::filesSent, processedhandler, &Processedhandler::startChecking);
}

MainWindow::~MainWindow()
{
    ClientInterface::sendClear();
    if (QDir("TEMP").exists())
        QDir("TEMP").removeRecursively();
    if (QFile("numprocessed.txt").exists())
        QFile("numprocessed.txt").remove();
    delete ui;
}

void MainWindow::saveImage()
{
    processedhandler->save(ui->preTransImagesList->currentRow());
}

void MainWindow::sendImage()
{
    preprocessedhandler->send();

    QMessageBox *messBox = new QMessageBox(this);
    messBox->setWindowTitle("Info");
    messBox->setText("Images sent to server");
    messBox->exec();
}

void MainWindow::updatePreTransImageList()
{
    ui->preTransImagesList->clear();
    ui->preTransImagesList->addItems(preprocessedhandler->getImagesNames());
}

void MainWindow::refreshRemainingTime()
{
    ui->remainingTimeEdit->setText(QString::number(processedhandler->getRemainingTime()/1000));
}

void MainWindow::updateProcessedList()
{
    ui->processedImagesList->clear();
    ui->processedImagesList->addItems(processedhandler->getImagesNames());
}
