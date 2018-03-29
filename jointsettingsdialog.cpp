#include "jointsettingsdialog.h"
#include "ui_jointsettingsdialog.h"
#include <QTimer>
#include <QDebug>
#include <QHeaderView>
#include <sstream>

#define MOTION_TEST
#define GEAR1 (1.0)
#define GEAR2 (1.0)
#define GEAR3 (1.0)
#define GEAR4 (1.0)
#define GEAR5 (1.0)
#define GEAR6 (1.0)
#define ROT1 (1)
#define ROT2 (1)
#define ROT3 (1)
#define ROT4 (1)
#define ROT5 (1)
#define ROT6 (1)

JointSettingsDialog::JointSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JointSettingsDialog)
{
    ui->setupUi(this);
    left_arm.resize(7);
    right_arm.resize(7);
    left_finger.resize(11);
    right_finger.resize(11);
    time.resize(3);

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

    QVector<double> gears;
    gears.push_back(GEAR1);
    gears.push_back(GEAR2);
    gears.push_back(GEAR3);
    gears.push_back(GEAR4);
    gears.push_back(GEAR5);
    gears.push_back(GEAR6);

    QVector<double> rot;
    rot.push_back(ROT1);
    rot.push_back(ROT2);
    rot.push_back(ROT3);
    rot.push_back(ROT4);
    rot.push_back(ROT5);
    rot.push_back(ROT6);

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

    pos_dial.resize(6);
    pos_dial[0] = ui->ppos1_dial;
    pos_dial[1] = ui->ppos2_dial;
    pos_dial[2] = ui->ppos3_dial;
    pos_dial[3] = ui->ppos4_dial;
    pos_dial[4] = ui->ppos5_dial;
    pos_dial[5] = ui->ppos6_dial;

    goalpos_dial.resize(6);
    goalpos_dial[0] = ui->goalpos1_dial;
    goalpos_dial[1] = ui->goalpos2_dial;
    goalpos_dial[2] = ui->goalpos3_dial;
    goalpos_dial[3] = ui->goalpos4_dial;
    goalpos_dial[4] = ui->goalpos5_dial;
    goalpos_dial[5] = ui->goalpos6_dial;

    pos_label.resize(6);
    pos_label[0] = ui->ppos1_label;
    pos_label[1] = ui->ppos2_label;
    pos_label[2] = ui->ppos3_label;
    pos_label[3] = ui->ppos4_label;
    pos_label[4] = ui->ppos5_label;
    pos_label[5] = ui->ppos6_label;

    goalpos_label.resize(6);
    goalpos_label[0] = ui->goalpos1_label;
    goalpos_label[1] = ui->goalpos2_label;
    goalpos_label[2] = ui->goalpos3_label;
    goalpos_label[3] = ui->goalpos4_label;
    goalpos_label[4] = ui->goalpos5_label;
    goalpos_label[5] = ui->goalpos6_label;

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

    ui->tableWidget->setColumnCount(13);

    QStringList horizontal_header;
    horizontal_header << "x (left)"     << "y (left)"   << "z (left)"
                      << "rx (left)"    << "ry (left)"  << "rz (left)"
                      << "x (right)"     << "y (right)"   << "z (right)"
                      << "rx (right)"    << "ry (right)"  << "rz (right)"
                      << "time";
    ui->tableWidget->setHorizontalHeaderLabels(horizontal_header);
    for(size_t i=0; i<ui->tableWidget->columnCount(); i++)
        ui->tableWidget->setColumnWidth(i,65);

    QStringList joint_horizontal_header;
    joint_horizontal_header << "Gear Ratio"
                            << "Rotation";
    ui->joint_settings_table->setColumnCount(2);
    ui->joint_settings_table->setRowCount(6);
    ui->joint_settings_table->setHorizontalHeaderLabels(joint_horizontal_header);
    for(size_t i=0; i<6; i++)
    {
        ui->joint_settings_table->setColumnWidth(i,75);
        ui->joint_settings_table->setItem(i,0,new QTableWidgetItem(QString::number(gears.at(i))));
        ui->joint_settings_table->setItem(i,1,new QTableWidgetItem(QString::number(rot.at(i))));
    }

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
        ui->tableWidget->setItem(row,12,new QTableWidgetItem(QString::number(ui->time_sbox->value())));
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
            if(row>=1)
            {
                if(ui->tableWidget->rowCount()>=2)
                {
                    QVector<double> frame1;
                    for(int i=0; i<12; i++)
                    {
                        auto value = ui->tableWidget->item(row-1,i)->data(0).toDouble();
                        frame1.push_back(value);
                    }
                    if(!motion_timer->isActive())
                    {
                        motion_ik0 = frame1;
                        motion_ik1 = frame;
                        time[0] = ui->tableWidget->item(row-1,12)->data(0).toDouble();
                        time[1] = ui->tableWidget->item(row,12)->data(0).toDouble();
                        time[2] = time[0];
                        motion_timer->start(33);
                    }
                }
            }
            else
                this->btnIkRequest(frame);
        }
    });

    connect(ui->remove_btn,&QPushButton::clicked,[=]
    {
        auto row = ui->tableWidget->currentRow();
        if(row>=0)
            ui->tableWidget->removeRow(row);
    });

    motion_timer = new QTimer(this);

    connect(motion_timer,SIGNAL(timeout()),this,SLOT(playMotion()));
    auto port_list = QSerialPortInfo::availablePorts();
    for(const QSerialPortInfo& s : port_list)
        ui->serial_comm_cbx->addItem(s.portName());

    connect(ui->connect_btn,&QPushButton::clicked,[=]
    {
        if(!connect_serial_cb)
            return;
        auto port = ui->serial_comm_cbx->currentText();
        auto res = connect_serial_cb(QString("/dev/%1").arg(port).toStdString());
        if(res)
        {
            ui->connect_btn->setText(QString("Disconnect"));
            ui->connect_btn->setEnabled(false);
        }
    });

    connect(ui->joint_settings_table,&QTableWidget::cellChanged,[=]
    {
        QVector<double> ratio;
        for(size_t i=0; i<6; i++)
            ratio.push_back(ui->tableWidget->itemAt(0,i)->data(0).toDouble());
        qDebug() << "gear :" << ratio;
    });

    connect(ui->torque1_checkbox,SIGNAL(toggled(bool)),this,SIGNAL(torqueRequest()));
    connect(ui->torque2_checkbox,SIGNAL(toggled(bool)),this,SIGNAL(torqueRequest()));
    connect(ui->torque3_checkbox,SIGNAL(toggled(bool)),this,SIGNAL(torqueRequest()));
    connect(ui->torque4_checkbox,SIGNAL(toggled(bool)),this,SIGNAL(torqueRequest()));
    connect(ui->torque5_checkbox,SIGNAL(toggled(bool)),this,SIGNAL(torqueRequest()));
    connect(ui->torque6_checkbox,SIGNAL(toggled(bool)),this,SIGNAL(torqueRequest()));

    connect(ui->goalpos1_dial,SIGNAL(valueChanged(int)),this,SIGNAL(goalPosRequest()));
    connect(ui->goalpos2_dial,SIGNAL(valueChanged(int)),this,SIGNAL(goalPosRequest()));
    connect(ui->goalpos3_dial,SIGNAL(valueChanged(int)),this,SIGNAL(goalPosRequest()));
    connect(ui->goalpos4_dial,SIGNAL(valueChanged(int)),this,SIGNAL(goalPosRequest()));
    connect(ui->goalpos5_dial,SIGNAL(valueChanged(int)),this,SIGNAL(goalPosRequest()));
    connect(ui->goalpos6_dial,SIGNAL(valueChanged(int)),this,SIGNAL(goalPosRequest()));

    for(size_t i=0; i<6; i++)
        connect(goalpos_dial[i],SIGNAL(valueChanged(int)),this,SLOT(setGoalPos()));

    setWindowTitle("Joint Settings");
}

void JointSettingsDialog::setJointFromController(std::vector<double> joints)
{
    ui->arm_select_slider->setValue(0);
    for(size_t i=0; i<joints.size(); i++)
        if(!std::isnan(joints.at(i)))
            arm_slider.at(i)->setValue(joints[i]);
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
#if 0
    qDebug() << ret;
#endif
    return ret;
}

QVector<double> JointSettingsDialog::gearRatio()
{
    QVector<double> gear;
    for(size_t i=0; i<ui->joint_settings_table->rowCount(); i++)
    {
        auto g = ui->joint_settings_table->itemAt(i,0)->data(0).toDouble();
        gear.push_back(g);
    }
    return gear;
}

QVector<double> JointSettingsDialog::rotation()
{
    QVector<double> rot;
    for(size_t i=0; i<ui->joint_settings_table->rowCount(); i++)
    {
        auto r = ui->joint_settings_table->itemAt(i,1)->data(0).toDouble();
        rot.push_back(r);
    }
    return rot;
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
#if 0
    qDebug() << ret;
#endif
    return ret;
}

QVector<bool> JointSettingsDialog::torque()
{
    QVector<bool> ret;
    ret.resize(6);
    ret[0] = ui->torque1_checkbox->isChecked();
    ret[1] = ui->torque2_checkbox->isChecked();
    ret[2] = ui->torque3_checkbox->isChecked();
    ret[3] = ui->torque4_checkbox->isChecked();
    ret[4] = ui->torque5_checkbox->isChecked();
    ret[5] = ui->torque6_checkbox->isChecked();
    return ret;
}

QVector<int> JointSettingsDialog::goalPos()
{
    QVector<int> ret;
    ret.push_back(ui->goalpos1_dial->value());
    ret.push_back(ui->goalpos2_dial->value());
    ret.push_back(ui->goalpos3_dial->value());
    ret.push_back(ui->goalpos4_dial->value());
    ret.push_back(ui->goalpos5_dial->value());
    ret.push_back(ui->goalpos6_dial->value());
    return ret;
}

void JointSettingsDialog::setPresentPos(std::vector<int> pos)
{
    for(size_t i=0; i<std::min(pos.size(),(size_t)6); i++)
    {
        pos_dial[i]->setValue(pos[i]);
        pos_label[i]->setText(QString::number(pos[i]));
    }
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

void JointSettingsDialog::playMotion()
{
    qDebug() << "play motion :"
             << "interval :" << motion_timer->interval()
             << "remaining time :" << motion_timer->remainingTime()
             << "time :" << time;
    time[2] += ((double)motion_timer->interval())/1000.0;
    if(time.at(2)>time.at(1))
        motion_timer->stop();
    emit updateMotion(motion_ik0,motion_ik1,time);
}

void JointSettingsDialog::setGoalPos()
{
    for(size_t i=0; i<6; i++)
        goalpos_label[i]->setText(QString::number(goalpos_dial[i]->value()));
}
