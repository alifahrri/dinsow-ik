#include <QApplication>
#include "mainwindow.h"
#include "dinsowkinematic.h"
#include "dinsowmotion.h"
#include "servocontroller.h"
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(dinsowbody);

    DinsowKinematic dinsow_kinematic;
    DinsowMotion dinsow_motion(dinsow_kinematic);
    ServoController controller;

    controller.dinsow = &dinsow_kinematic;
    if(!controller.open("/dev/ttyUSB0"))
    {
        std::cout << "failed to open serial!\n";
        return -1;
    }

    MainWindow w;
    w.getWidget3D()->passDinsow(&dinsow_kinematic,&dinsow_motion);
    w.showMaximized();

    controller.start();

    return a.exec();
}
