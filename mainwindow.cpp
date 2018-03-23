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

    connect(joint_dialog,&JointSettingsDialog::ikRequest,[=]
    {
        auto left_frame = joint_dialog->leftArmIk();
        auto right_frame = joint_dialog->rightArmIk();
        left_frame.append(right_frame);
        widget3d->modifier->dinsowIK(left_frame);
    });

    connect(joint_dialog,&JointSettingsDialog::btnIkRequest,[=](QVector<double> frame)
    {
        widget3d->modifier->dinsowIK(frame);
    });

    connect(joint_dialog,&JointSettingsDialog::updateMotion,[=](QVector<double> f0, QVector<double> f1, QVector<double> time)
    {
#if 1
        qDebug() << "[updateMotion]" << time;
#endif
        widget3d->modifier->dinsowMotionIK(f0,f1,time);
    });

    connect(widget3d->modifier,SIGNAL(jointUpdateIK(QVector<double>,QVector<double>)),
            joint_dialog,SLOT(setJointsFromIK(QVector<double>,QVector<double>)));

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
