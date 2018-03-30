#ifndef JOINTSETTINGSDIALOG_H
#define JOINTSETTINGSDIALOG_H

#include <functional>
#include <QDialog>
#include <QDial>
#include <QLabel>
#include <vector>
#include <QtSerialPort/QSerialPortInfo>
#include "servocontroller.h"

class QSlider;

namespace Ui {
class JointSettingsDialog;
}

class JointSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    typedef std::function<bool(std::string)> ConnectSerialCallback;

public:
    explicit JointSettingsDialog(QWidget *parent = 0);
    void setJointFromController(std::vector<double> joints);
    void setPresentPos(std::vector<int> pos);
    void showEEPROMSettings(const ServoController::EEPROMSettings &settings, int id);
    bool eepromSettings(int id, ServoController::EEPROMSettings &settings);
    QVector<double> fingerJointValues();
    QVector<double> jointValues();
    QVector<double> rightArmIk();
    QVector<double> leftArmIk();
    QVector<double> gearRatio();
    QVector<int> rotation();
    QVector<bool> torque();
    QVector<int> goalPos();
    ~JointSettingsDialog();

public:
    ConnectSerialCallback connect_serial_cb = nullptr;

public slots:
    void setJointsFromIK(QVector<double> joints, QVector<double> hands);

private slots:
    void playMotion();
    void setGoalPos();

signals:
    void updateMotion(QVector<double>,QVector<double>,QVector<double>);
    void jointSettingsUpdate(QVector<double>);
    void saveJointSettings(QVector<double>);
    void btnIkRequest(QVector<double>);
    void eepromWriteRequest();
    void eepromReadRequest();
    void jointValueChanged();
    void controllerRequest();
    void goalPosRequest();
    void rightIkRequest();
    void torqueRequest();
    void leftIkRequest();
    void ikRequest();

private:
    Ui::JointSettingsDialog *ui;
    QVector<QSlider*> arm_slider;
    QVector<QSlider*> finger_slider;
    QVector<QLabel*> goalpos_label;
    QVector<QLabel*> pos_label;
    QVector<QDial*> goalpos_dial;
    QVector<QDial*> pos_dial;
    QVector<double> left_arm;
    QVector<double> right_arm;
    QVector<double> left_finger;
    QVector<double> right_finger;
    QVector<double> motion_ik0;
    QVector<double> motion_ik1;
    QVector<double> time;
    QVector<double> gears;
    QVector<int> rot;
    QTimer *motion_timer;
};

#endif // JOINTSETTINGSDIALOG_H
