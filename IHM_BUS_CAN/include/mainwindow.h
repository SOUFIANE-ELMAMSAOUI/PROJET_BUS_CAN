#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "socketcan_cpp.h"
#include "objectgl.h"
#include <QtOpenGL>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    void openCANPort();
    void receiveCANMessage();

    // Handler functions for different sensor data
    void handleSensorData(const scpp::CanFrame& fr);
    void handleAngleData(const scpp::CanFrame& fr);
    void handleWindSpeedData(const scpp::CanFrame& fr);

    ~MainWindow();

private slots:
    void onTimer_Tick();
    void on_setSpeed_clicked();
    void on_DistanceRadio_clicked(bool checked = false);
    void on_BrightnessRadio_clicked(bool checked = false);

private:
    scpp::SocketCan socket_can;
    QTimer *timer_tick;
    ObjectOpenGL *Object_GL;
    Ui::MainWindow *ui;

    // Send motor speed command
    void sendMotorCommand(uint8_t speed);

    // Send sensor mode commands
    void sendDistanceModeCommand();
    void sendBrightnessModeCommand();
};
#endif // MAINWINDOW_H
