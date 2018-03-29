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

    MainWindow w;
    w.getWidget3D()->passDinsow(&dinsow_kinematic,&dinsow_motion);
    w.showMaximized();

    controller.serial_cb = [&]
    {
        auto ppos = controller.presentPos();
        auto ppos_deg = controller.presentPosDeg();
        w.joint_dialog->setPresentPos(ppos);
        w.joint_dialog->setJointFromController(ppos_deg);
    };

    w.joint_dialog->connect_serial_cb = [&](std::string port)
    {
        auto res = controller.open(port);
        return res;
    };

    auto rot = w.joint_dialog->rotation().toStdVector();
    auto gear = w.joint_dialog->gearRatio().toStdVector();

    controller.setGearRatio(gear);

    QObject::connect(w.joint_dialog,&JointSettingsDialog::torqueRequest,[&]
    {
        auto tqs = w.joint_dialog->torque().toStdVector();
        controller.setTorque(tqs);
    });

    QObject::connect(w.joint_dialog,&JointSettingsDialog::goalPosRequest,[&]
    {
        auto goal_pos = w.joint_dialog->goalPos().toStdVector();
        controller.setGoalPos(goal_pos);
    });

    controller.start();

    return a.exec();
}
