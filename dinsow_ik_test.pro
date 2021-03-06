#-------------------------------------------------
#
# Project created by QtCreator 2018-03-06T10:45:27
#
#-------------------------------------------------

QT       += core gui network
QT += 3dcore 3drender 3dinput 3dlogic 3dextras serialport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

#CONFIG += console
TARGET = dinsow_ik_test
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

LIBS += -L/usr/local/lib -lprotobuf

SOURCES += main.cpp\
        mainwindow.cpp \
    widget3d.cpp \
    scenemodifier.cpp \
    linkentity.cpp \
    jointsettingsdialog.cpp \
    jointentity.cpp \
    dinsowkinematic.cpp \
    meshentity.cpp \
    dinsowmotion.cpp \
    servocontroller.cpp \
    client.cpp \
    message.pb.cc

HEADERS  += mainwindow.h \
    widget3d.h \
    scenemodifier.h \
    linkentity.h \
    jointsettingsdialog.h \
    jointentity.h \
    dinsowkinematic.h \
    meshentity.h \
    dinsowmotion.h \
    servocontroller.h \
    utility.hpp \
    servolimit.h \
    client.h \
    message.pb.h

FORMS    += mainwindow.ui \
    jointsettingsdialog.ui

LIBS += -lorocos-kdl -ldxl_x64_cpp -lrt

RESOURCES += \
    dinsowbody.qrc

INCLUDEPATH += /usr/include/eigen3

#QMAKE_CXXFLAGS_RELEASE -= -O1
#QMAKE_CXXFLAGS_RELEASE -= -O2
#QMAKE_CXXFLAGS_RELEASE += -O3

