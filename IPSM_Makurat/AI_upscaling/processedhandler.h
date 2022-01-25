#ifndef PROCESSEDHANDLER_H
#define PROCESSEDHANDLER_H

#include "imagehandler.h"

class Processedhandler : public ImageHandler
{
    Q_OBJECT
public:
    Processedhandler(QScrollArea *scrollArea, QWidget *parent = nullptr);

    QStringList getImagesNames();
    int getRemainingTime();
    void save(int index);
    void saveAll();

public slots:
    void startChecking();
    void forceCheck();

signals:
    void refreshRemainingTime();

private slots:
    void checkForAvailableImages();

private:
    void refreshSignal();

    QTimer *timer;
    QTimer *refreshTimer;
};

#endif // PROCESSEDHANDLER_H
