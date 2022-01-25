#include "preprocessedhandler.h"
#include "clientinterface.h"
#include <QFileDialog>

Preprocessedhandler::Preprocessedhandler(QScrollArea *scrollArea, QWidget *parent) : ImageHandler(scrollArea, parent)
{

}

QStringList Preprocessedhandler::getImagesNames()
{
    QStringList imagesNames;

    for (int i = 0; i < imageList.length(); i++)
    {
        QFileInfo file(filesNames[i]);
        imagesNames.append(file.fileName());
    }
    return imagesNames;
}

void Preprocessedhandler::send()
{
    ClientInterface::send(filesNames);
    ClientInterface::beginProcessing();
    emit filesSent();
}

void Preprocessedhandler::open()
{
    filesNames = QFileDialog::getOpenFileNames(
                this,
                "Choose images",
                "C://",
                "JPEG files (*.jpg)");

    for (int i = 0; i < filesNames.length(); i++)
    {
         QImage image;
         if (image.load(filesNames[i]))
            imageList.append(image);
    }

    emit filesReady();
}
