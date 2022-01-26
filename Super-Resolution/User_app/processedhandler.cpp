#include "processedhandler.h"
#include <QTimer>
#include <QFileDialog>
#include "clientinterface.h"

Processedhandler::Processedhandler(QScrollArea *scrollArea, QWidget *parent) : ImageHandler(scrollArea, parent)
{
    timer = new QTimer(this);
    refreshTimer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Processedhandler::checkForAvailableImages);
    connect(refreshTimer, &QTimer::timeout, this, &Processedhandler::refreshSignal);
}

QStringList Processedhandler::getImagesNames()
{
    return filesNames;
}

int Processedhandler::getRemainingTime()
{
    return timer->remainingTime();
}

void Processedhandler::save(int index)
{
    QString saveFileName = QFileDialog::getSaveFileName(
                this,
                "Save file",
                "C://",
                "JPEG files (*jpg)");

    if (saveFileName != "")
    {
        QFileInfo fileInfo(saveFileName);

        if (fileInfo.suffix() != "jpg")
            saveFileName.append(".jpg");

        imageList[index].save(saveFileName);
    }
}

void Processedhandler::saveAll()
{
    QString dir = QFileDialog::getExistingDirectory(this,
                                                    tr("Open directory"),
                                                    "C://",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    if (dir != "")
    {
        for (int i = 0; i < filesNames.length(); i++)
        {
            QString saveFileName = dir + "//" + filesNames[i];
            imageList[i].save(saveFileName);
        }
    }
}

void Processedhandler::startChecking()
{
    timer->start(30000);
    refreshTimer->start(1000);
}

void Processedhandler::forceCheck()
{
    checkForAvailableImages();
    timer->start(30000);
    refreshSignal();
}

void Processedhandler::checkForAvailableImages()
{
    int availableImages = ClientInterface::checkForAvailableImages();
    if (imageList.count() < availableImages)
    {
        for (int i = imageList.count(); i < availableImages; i++)
        {
            imageList.append(ClientInterface::getProcessedImage(i));
            QString name = QString::number(i) + ".jpg";
            filesNames.append(name);

        }
        emit filesReady();
    }
}

void Processedhandler::refreshSignal()
{
    emit refreshRemainingTime();
}
