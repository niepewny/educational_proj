#include "imagehandler.h"
#include <QFileDialog>
#include <QMenu>
#include "curl/include/curl/curl.h"
#include <cstdlib>
#include "clientinterface.h"


ImageHandler::ImageHandler(QScrollArea *scrollArea, QWidget *parent) : QWidget(parent)
{
    imageLabel = new QLabel(scrollArea);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);
}

QLabel* ImageHandler::getImageLabel()
{
    return imageLabel;
}

void ImageHandler::display(int index)
{
    imageLabel->setPixmap(QPixmap::fromImage(imageList[index]));
}
