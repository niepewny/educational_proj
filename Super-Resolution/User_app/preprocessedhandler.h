#ifndef PREPROCESSEDHANDLER_H
#define PREPROCESSEDHANDLER_H

#include "imagehandler.h"


class Preprocessedhandler : public ImageHandler
{
    Q_OBJECT
public:
    Preprocessedhandler(QScrollArea *scrollArea, QWidget *parent = nullptr);

    QStringList getImagesNames();
    void send();

public slots:
    void open();
};

#endif // PREPROCESSEDHANDLER_H
