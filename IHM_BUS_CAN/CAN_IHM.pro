# -------------------------------------------------
# Project created by QtCreator 2011-05-31T11:30:38
# -------------------------------------------------
QT +=           opengl
QT += core gui widgets opengl

TARGET =        mpu9250-OpenGl

TEMPLATE =      app

SOURCES +=      src/main.cpp
SOURCES +=      src/mainwindow.cpp
SOURCES +=      src/objectgl.cpp
SOURCES +=
SOURCES +=
SOURCES +=

HEADERS +=      \
    include/mainwindow.h
HEADERS +=      include/objectgl.h
HEADERS +=
HEADERS +=
HEADERS +=

INCLUDEPATH +=  src
INCLUDEPATH +=  include


OBJECTS_DIR = tmp/
MOC_DIR = tmp/
DESTDIR = bin/

FORMS += \
    mainwindow.ui


# Include resources
RESOURCES += \
    resources.qrc

OTHER_FILES += \
    style.qss


SOURCES += src/socketcan_cpp.cpp
HEADERS += include/socketcan_cpp.h include/socketcan_cpp_export.h


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
