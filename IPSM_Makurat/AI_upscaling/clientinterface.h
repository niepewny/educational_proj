#ifndef CLIENTINTERFACE_H
#define CLIENTINTERFACE_H

#include <QObject>

class ClientInterface : public QObject
{
    Q_OBJECT
public:
    explicit ClientInterface(QObject *parent = nullptr);

    static void send(QStringList filesNames);
    static int checkForAvailableImages();
    static void beginProcessing();
    static QImage getProcessedImage(int index);
    static void sendClear();

private:
    static void sendString(QString text);

    static QString hostName;
};

#endif // CLIENTINTERFACE_H
