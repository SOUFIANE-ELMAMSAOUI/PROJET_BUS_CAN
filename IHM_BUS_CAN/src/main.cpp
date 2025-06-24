#include "mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Load stylesheet
    QFile styleFile(":/style.qss");
    if (styleFile.open(QFile::ReadOnly)) {
        QString styleSheet = styleFile.readAll();
        a.setStyleSheet(styleSheet);
        styleFile.close();
    } else {
        qDebug() << "Unable to open stylesheet file";
    }

    MainWindow w;
    w.setWindowTitle("MPU9250 Orientation Viewer");
    w.show();

    return a.exec();
}
