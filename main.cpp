#include <QApplication>
#include "mainwindow.h"
#include "dinsowkinematic.h"
#include "dinsowmotion.h"
#include "servocontroller.h"
#include "scenemodifier.h"
#include "client.h"
#include <iostream>

#define PORT_TARGET 55284

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(dinsowbody);

    DinsowKinematic dinsow_kinematic;
    DinsowMotion dinsow_motion(dinsow_kinematic);
    ServoController controller;

    controller.dinsow = &dinsow_kinematic;

    MainWindow w;
    TargetClient target_client(PORT_TARGET, &w);

    w.getWidget3D()->passDinsow(&dinsow_kinematic,&dinsow_motion);
    w.showMaximized();

    controller.serial_cb = [&]
    {
        auto ppos = controller.presentPos();
        auto ppos_deg = controller.presentPosDeg();
        w.joint_dialog->setPresentPos(ppos);
//        dinsow_kinematic.setJoints();
#if 0
        w.joint_dialog->setJointFromController(ppos_deg);
#endif
        qDebug() << "[callback] serial :" << ppos_deg;
    };

    w.joint_dialog->connect_serial_cb = [&](std::string port)
    {
        auto res = controller.open(port);
        return res;
    };

    auto rot = w.joint_dialog->rotation().toStdVector();
    auto gear = w.joint_dialog->gearRatio().toStdVector();

    controller.setGearRatio(gear);
    controller.setRotation(rot);

    QObject::connect(w.joint_dialog,&JointSettingsDialog::torqueRequest,[&]
    {
        auto tqs = w.joint_dialog->torque().toStdVector();
        controller.setTorque(tqs);
    });

#if 0
    QObject::connect(w.joint_dialog,&JointSettingsDialog::goalPosRequest,[&]
    {
        auto goal_pos = w.joint_dialog->goalPos().toStdVector();
        controller.setGoalPos(goal_pos);
    });
#endif

    QObject::connect(&target_client, &TargetClient::ikRequest, [&](double x, double y, double z)
    {
      w.widget3d->modifier->setTarget(x,y,z);
    });

    QObject::connect(w.joint_dialog,&JointSettingsDialog::eepromReadRequest,[&]{
        ServoController::EEPROMSettings eeprom[6];
        for(int i=0; i<6; i++)
            eeprom[i] = controller.readEEPROM(i+1);
        for(int i=0; i<6; i++)
        {
            w.joint_dialog->showEEPROMSettings(eeprom[i],i);
            std::cout << "[READ EEPROM] ID : " << i+1 << " " << eeprom[i].str() << std::endl;
        }
    });

    QObject::connect(w.joint_dialog,&JointSettingsDialog::eepromWriteRequest,[&]
    {
        ServoController::EEPROMSettings eeprom[6];
        bool valid[6];
        bool comm[6];
        for(int i=0; i<6; i++)
            valid[i] = w.joint_dialog->eepromSettings(i,eeprom[i]);
        for(int i=0; i<6; i++)
            if(valid[i])
                comm[i] = controller.writeEEPROM(eeprom[i],i+1);
        for(int i=0; i<6; i++)
            std::cout << "[WRITE EEPROM] ID : " << i+1 << (valid[i] ? " ok " : " !ok ") << eeprom[i].str() << (comm[i] ? " success" : " failed") << std::endl;
        w.joint_dialog->eepromReadRequest();
    });

    controller.start();

    return a.exec();
}
