 #ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "preprocessedhandler.h"
#include "processedhandler.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void saveImage();
    void sendImage();
    void updatePreTransImageList();
    void refreshRemainingTime();

    void updateProcessedList();

private:
    Ui::MainWindow *ui;
    Preprocessedhandler *preprocessedhandler;
    Processedhandler *processedhandler;
};
#endif // MAINWINDOW_H
