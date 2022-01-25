#ifndef IMAGEHANDLER_H
#define IMAGEHANDLER_H

#include <QWidget>
#include <QLabel>
#include <QAction>
#include <QScrollBar>
#include <QScrollArea>

class ImageHandler : public QWidget
{
    Q_OBJECT
public:
    explicit ImageHandler(QScrollArea *scrollArea, QWidget *parent = nullptr);

    QLabel* getImageLabel();
    virtual QStringList getImagesNames() = 0;

public slots:
    void display(int index);

signals:
    void filesReady();
    void filesSent();

protected:
    QStringList filesNames;
    QList<QImage> imageList;
    QLabel *imageLabel = nullptr;
};

#endif // IMAGEHANDLER_H
