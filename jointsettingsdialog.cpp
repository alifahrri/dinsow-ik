#include "jointsettingsdialog.h"
#include "ui_jointsettingsdialog.h"
#include <QDebug>
#include <QHeaderView>

JointSettingsDialog::JointSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JointSettingsDialog)
{
    ui->setupUi(this);
    left_arm.resize(7);
    right_arm.resize(7);
    left_finger.resize(11);
    right_finger.resize(11);

    for(int i=0; i<7; i++)
    {
        left_arm[i] = 0.0;
        right_arm[i] = 0.0;
    }
    for(int i=0; i<11; i++)
    {
        left_finger[i] = 0.0;
        right_finger[i] = 0.0;
    }

    arm_slider.resize(7);
    arm_slider[0] = ui->slider1;
    arm_slider[1] = ui->slider2;
    arm_slider[2] = ui->slider3;
    arm_slider[3] = ui->slider4;
    arm_slider[4] = ui->slider5;
    arm_slider[5] = ui->slider6;
    arm_slider[6] = ui->slider7;

    finger_slider.resize(11);
    finger_slider[0] = ui->finger_slider1;
    finger_slider[1] = ui->finger_slider2;
    finger_slider[2] = ui->finger_slider3;
    finger_slider[3] = ui->finger_slider4;
    finger_slider[4] = ui->finger_slider5;
    finger_slider[5] = ui->finger_slider6;
    finger_slider[6] = ui->finger_slider7;
    finger_slider[7] = ui->finger_slider8;
    finger_slider[8] = ui->finger_slider9;
    finger_slider[9] = ui->finger_slider10;
    finger_slider[10] = ui->finger_slider11;


    for(auto& s : arm_slider)
        connect(s,SIGNAL(valueChanged(int)),this,SIGNAL(jointValueChanged()));

    for(auto& s : finger_slider)
        connect(s,SIGNAL(valueChanged(int)),this,SIGNAL(jointValueChanged()));

    for(int i=0; i<7; i++)
    {
        connect(arm_slider[i],&QSlider::valueChanged,[=](int value)
        {
            auto pos1 = ui->arm_select_slider->value();
            if(!pos1)
                left_arm[i] = value;
            else
                right_arm[i] = value;
        });
    }
    for(int i=0; i<11; i++)
    {
        connect(finger_slider[i],&QSlider::valueChanged,[=](int value)
        {
            auto pos1 = ui->arm_select_slider->value();
            if(!pos1)
                left_finger[i] = value;
            else
                right_finger[i] = value;
        });
    }

    ui->slider1->setEnabled(false);

    connect(ui->reset_btn,&QPushButton::clicked,[=]
    {
        for(auto& j : left_arm)
            j = 0.0;
        for(auto& j : right_arm)
            j = 0.0;
        for(auto& j : left_finger)
            j = 0.0;
        for(auto& j : right_finger)
            j = 0.0;

        for(auto & slider : arm_slider)
            slider->setValue(0);
        for(auto & slider : finger_slider)
            slider->setValue(0);
    });

    connect(ui->arm_select_slider,&QSlider::valueChanged,[=](int pos)
    {
        if(pos)
        {
            for(int i=0; i<7; i++)
                arm_slider.at(i)->setValue((int)right_arm.at(i));
            for(int i=0; i<11; i++)
                finger_slider.at(i)->setValue((int)right_finger.at(i));
        }
        else
        {
            for(int i=0; i<7; i++)
                arm_slider.at(i)->setValue((int)left_arm.at(i));
            for(int i=0; i<11; i++)
                finger_slider.at(i)->setValue((int)left_finger.at(i));
        }
    });

#if 0
    connect(ui->x_ik_left_slider,SIGNAL(valueChanged(int)),this,SIGNAL(leftIkRequest()));
    connect(ui->y_ik_left_slider,SIGNAL(valueChanged(int)),this,SIGNAL(leftIkRequest()));
    connect(ui->z_ik_left_slider,SIGNAL(valueChanged(int)),this,SIGNAL(leftIkRequest()));
    connect(ui->rx_ik_left_dial,SIGNAL(valueChanged(int)),this,SIGNAL(leftIkRequest()));
    connect(ui->ry_ik_left_dial,SIGNAL(valueChanged(int)),this,SIGNAL(leftIkRequest()));
    connect(ui->rz_ik_left_dial,SIGNAL(valueChanged(int)),this,SIGNAL(leftIkRequest()));

    connect(ui->x_ik_right_slider,SIGNAL(valueChanged(int)),this,SIGNAL(rightIkRequest()));
    connect(ui->y_ik_right_slider,SIGNAL(valueChanged(int)),this,SIGNAL(rightIkRequest()));
    connect(ui->z_ik_right_slider,SIGNAL(valueChanged(int)),this,SIGNAL(rightIkRequest()));
    connect(ui->rx_ik_right_dial,SIGNAL(valueChanged(int)),this,SIGNAL(rightIkRequest()));
    connect(ui->ry_ik_right_dial,SIGNAL(valueChanged(int)),this,SIGNAL(rightIkRequest()));
    connect(ui->rz_ik_right_dial,SIGNAL(valueChanged(int)),this,SIGNAL(rightIkRequest()));
#else
    connect(ui->x_ik_left_slider,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->y_ik_left_slider,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->z_ik_left_slider,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->rx_ik_left_dial,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->ry_ik_left_dial,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->rz_ik_left_dial,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));

    connect(ui->x_ik_right_slider,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->y_ik_right_slider,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->z_ik_right_slider,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->rx_ik_right_dial,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->ry_ik_right_dial,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
    connect(ui->rz_ik_right_dial,SIGNAL(valueChanged(int)),this,SIGNAL(ikRequest()));
#endif

    ui->tableWidget->setColumnCount(12);

    QStringList horizontal_header;
    horizontal_header << "x (left)"     << "y (left)"   << "z (left)"
                      << "rx (left)"    << "ry (left)"  << "rz (left)"
                      << "x (right)"     << "y (right)"   << "z (right)"
                      << "rx (right)"    << "ry (right)"  << "rz (right)";
    ui->tableWidget->setHorizontalHeaderLabels(horizontal_header);
    for(size_t i=0; i<12; i++)
        ui->tableWidget->setColumnWidth(i,65);

    connect(ui->add_btn,&QPushButton::clicked,[=]
    {
        auto left_frame = leftArmIk();
        auto right_frame = rightArmIk();
        auto& frame = left_frame;
        frame.append(right_frame);
        auto row = ui->tableWidget->rowCount();
        ui->tableWidget->setRowCount(row+1);
        for(size_t i=0; i<12; i++)
            ui->tableWidget->setItem(row,i,new QTableWidgetItem(QString::number(frame[i])));
    });

    connect(ui->set_btn,&QPushButton::clicked,[=]
    {
        auto row = ui->tableWidget->currentRow();
        if(row>=0)
        {
            QVector<double> frame;
            for(int i=0; i<12; i++)
            {
                auto value = ui->tableWidget->item(row,i)->data(0).toDouble();
                frame.push_back(value);
            }
            this->btnIkRequest(frame);
        }
    });

    connect(ui->remove_btn,&QPushButton::clicked,[=]
    {
        auto row = ui->tableWidget->currentRow();
        if(row>=0)
            ui->tableWidget->removeRow(row);
    });

    setWindowTitle("Joint Settings");
}

QVector<double> JointSettingsDialog::jointValues()
{
    QVector<double> ret;
    ret.append(left_arm);
    ret.append(right_arm);
    return ret;
}

QVector<double> JointSettingsDialog::fingerJointValues()
{
    QVector<double> ret;
    ret.append(left_finger);
    ret.append(right_finger);
    return ret;
}

QVector<double> JointSettingsDialog::leftArmIk()
{
    QVector<double> ret;
    ret.push_back(ui->z_ik_left_slider->value()/10.0);
    ret.push_back(ui->x_ik_left_slider->value()/10.0);
    ret.push_back(ui->y_ik_left_slider->value()/10.0);
    ret.push_back(ui->rx_ik_left_dial->value()/10.0);
    ret.push_back(ui->ry_ik_left_dial->value()/10.0);
    ret.push_back(ui->rz_ik_left_dial->value()/10.0);
    qDebug() << ret;
    return ret;
}

QVector<double> JointSettingsDialog::rightArmIk()
{
    QVector<double> ret;
    ret.push_back(ui->z_ik_right_slider->value()/10.0);
    ret.push_back(ui->x_ik_right_slider->value()/10.0);
    ret.push_back(ui->y_ik_right_slider->value()/10.0);
    ret.push_back(ui->rx_ik_right_dial->value()/10.0);
    ret.push_back(ui->ry_ik_right_dial->value()/10.0);
    ret.push_back(ui->rz_ik_right_dial->value()/10.0);
    qDebug() << ret;
    return ret;
}

JointSettingsDialog::~JointSettingsDialog()
{
    delete ui;
}

void JointSettingsDialog::setJointsFromIK(QVector<double> joints, QVector<double> hands)
{
    for(size_t i=1; i<7; i++)
        left_arm[i] = joints[i];
    for(size_t i=1; i<7; i++)
        right_arm[i] = joints[i+7];
    for(size_t i=0; i<11; i++)
        left_finger[i] = hands[i];
    for(size_t i=0; i<11; i++)
        right_finger[i] = hands[i+11];
    auto pos = ui->arm_select_slider->value();
    if(pos)
    {
        for(int i=0; i<7; i++)
            arm_slider.at(i)->setValue((int)right_arm.at(i));
        for(int i=0; i<11; i++)
            finger_slider.at(i)->setValue((int)right_finger.at(i));
    }
    else
    {
        for(int i=0; i<7; i++)
            arm_slider.at(i)->setValue((int)left_arm.at(i));
        for(int i=0; i<11; i++)
            finger_slider.at(i)->setValue((int)left_finger.at(i));
    }
}
