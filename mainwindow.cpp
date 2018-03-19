#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scenemodifier.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    widget3d(new Widget3D),
    joint_dialog(new JointSettingsDialog(this))
{
    ui->setupUi(this);
    setCentralWidget(widget3d->getContainer());

    connect(joint_dialog,&JointSettingsDialog::jointValueChanged,[=]
    {
        auto joints = joint_dialog->jointValues();
        auto finger_joints = joint_dialog->fingerJointValues();
        widget3d->modifier->applyFK(joints,finger_joints);
    });

    connect(joint_dialog,&JointSettingsDialog::leftIkRequest,[=]
    {
        auto frame = joint_dialog->leftArmIk();
        widget3d->modifier->dinsowIK(frame);
    });

    setWindowTitle("Widget3D");

    joint_dialog->show();
}

Widget3D *MainWindow::getWidget3D()
{
    return widget3d;
}

MainWindow::~MainWindow()
{
    delete ui;
}
