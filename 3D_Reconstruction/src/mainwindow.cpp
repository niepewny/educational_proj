#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include "PointFinder.h"
#include "PointReconstructor.h"
#include <QMessageBox>

QString MainWindow::dir;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    reconstManager = new ReconstManager;

    connect(reconstManager, &ReconstManager::finished, this, &MainWindow::threadFinished);

    connect(ui->ChooseDirPushButton, &QPushButton::released, this, &MainWindow::chooseDir);
    connect(ui->runPushButton, &QPushButton::released, this, &MainWindow::runButtonClicked);
    connect(ui->gaussSigmaLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->gaussSizeLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->orbScaleFacLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->orbNLevelLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->orbEdgeThLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->orbPatchSizLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->orbFastThLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->orbFirstLevLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->gaussSigmaLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->pointZMinLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->pointZMaxLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->pointRoiSizLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->pointMaxErrLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->filtRadLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);
    connect(ui->filtMinNeighLineEdit, &QLineEdit::textChanged, this, &MainWindow::checkInput);

    ui->pointFindGroupBox->setEnabled(0);
    ui->pointRecGroupBox->setEnabled(0);
    ui->filtGroupBox->setEnabled(0);
    ui->runPushButton->setEnabled(0);
    }

MainWindow::~MainWindow()
{
    delete reconstManager;
    delete ui;
}

void MainWindow::chooseDir()
{
    dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"));
    ui->pointFindGroupBox->setEnabled(1);
    ui->pointRecGroupBox->setEnabled(1);
    ui->filtGroupBox->setEnabled(1);

    ui->runPushButton->setEnabled(1);
}

void MainWindow::checkInput()
{
    bool flag = true;
    flag &= !ui->gaussSigmaLineEdit->text().isEmpty();
    flag &= !ui->gaussSizeLineEdit->text().isEmpty();
    flag &= !ui->orbScaleFacLineEdit->text().isEmpty();
    flag &= !ui->orbNLevelLineEdit->text().isEmpty();
    flag &= !ui->orbEdgeThLineEdit->text().isEmpty();
    flag &= !ui->orbPatchSizLineEdit->text().isEmpty();
    flag &= !ui->orbFastThLineEdit->text().isEmpty();
    flag &= !ui->orbFirstLevLineEdit->text().isEmpty();
    flag &= !ui->gaussSigmaLineEdit->text().isEmpty();
    flag &= !ui->pointZMinLineEdit->text().isEmpty();
    flag &= !ui->pointZMaxLineEdit->text().isEmpty();
    flag &= !ui->pointRoiSizLineEdit->text().isEmpty();
    flag &= !ui->pointMaxErrLineEdit->text().isEmpty();
    flag &= !ui->filtRadLineEdit->text().isEmpty();
    flag &= !ui->filtMinNeighLineEdit->text().isEmpty();

    ui->runPushButton->setEnabled(flag);

}

void MainWindow::runButtonClicked()
{
    if (!dir.isEmpty())
    {
        if (reconstManager->cloudMaker == nullptr)
        {
            reconstManager->cloudMaker = new CloudMaker(dir.toStdString());
        }

        ui->runPushButton->setEnabled(0);
        ui->runPushButton->setText("Running...");

        PointFinder::setGaussianSize(ui->gaussSizeLineEdit->text().toInt());
        PointFinder::setGaussianSigma(ui->gaussSigmaLineEdit->text().toDouble());
        PointFinder::setORBscaleFactor(ui->orbScaleFacLineEdit->text().toFloat());
        PointFinder::setORBnLevels(ui->orbNLevelLineEdit->text().toInt());
        PointFinder::setORBedgeThreshold(ui->orbEdgeThLineEdit->text().toInt());
        PointFinder::setORBpatchSize(ui->orbPatchSizLineEdit->text().toInt());
        PointFinder::setORBfastThreshold(ui->orbFastThLineEdit->text().toInt());
        PointFinder::setORBfirstLevel(ui->orbFirstLevLineEdit->text().toInt());

        PointReconstructor::setZmin(ui->pointZMinLineEdit->text().toFloat());
        PointReconstructor::setZmax(ui->pointZMaxLineEdit->text().toFloat());
        PointReconstructor::setROIsize(ui->pointRoiSizLineEdit->text().toInt());
        PointReconstructor::setMatcherMaxError(ui->pointMaxErrLineEdit->text().toInt());

        reconstManager->cloudMaker->filter.setRadiusSearch(ui->filtRadLineEdit->text().toDouble());
        reconstManager->cloudMaker->filter.setMinNeighborsInRadius(ui->filtMinNeighLineEdit->text().toInt());

        reconstManager->start();

    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Something went wrong.");
        msgBox.exec();
    }
}

void MainWindow::threadFinished()
{
    ui->runPushButton->setText("Run");
    ui->runPushButton->setEnabled(1);
}
