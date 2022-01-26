#include "clientinterface.h"
#include <QFile>
#include <QImage>
#include <QDir>
#include <windows.h>


QString ClientInterface::hostName = "http://localhost:8000/";

ClientInterface::ClientInterface(QObject *parent) : QObject(parent)
{

}

void ClientInterface::send(QStringList filesNames)
{
    for (int i = 0; i < filesNames.length(); i++)
    {
        QString command = "curl -F ";
        command.append("\"file=@").append(filesNames[i]).append("\" ");
        command.append(hostName);
        command.append("send");
        sendString(command);
    }
}

int ClientInterface::checkForAvailableImages()
{
    QString command = "curl " + hostName + "numprocessed -o numprocessed.txt";
    sendString(command);
    QFile fileNumProcessed("numprocessed.txt");
    if (!fileNumProcessed.open(QIODevice::ReadOnly | QIODevice::Text))
        return -1;
    return fileNumProcessed.readLine().toInt();
}

void ClientInterface::beginProcessing()
{
    QString command = "curl " + hostName + "process";
    sendString(command);
}

QImage ClientInterface::getProcessedImage(int index)
{
    if (!QDir("TEMP").exists())
        QDir(".").mkdir("TEMP");
    QString filePath = "TEMP/" + QString::number(index) + ".jpg";
    QString command = "curl " + hostName + "getprocessed/" +
            QString::number(index) + " -o " + filePath;
    sendString(command);
    return QImage(filePath);
}

void ClientInterface::sendClear()
{
    QString command = "curl " + hostName + "clear";
    sendString(command);
}


void ClientInterface::sendString(QString text)
{
    QByteArray temp = text.toLocal8Bit();
    char *command = temp.data();
    //WinExec(command, SW_HIDE);
    system(command);
}
