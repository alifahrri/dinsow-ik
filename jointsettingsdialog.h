#ifndef JOINTSETTINGSDIALOG_H
#define JOINTSETTINGSDIALOG_H

#include <QDialog>

class QSlider;

namespace Ui {
class JointSettingsDialog;
}

class JointSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit JointSettingsDialog(QWidget *parent = 0);
    QVector<double> jointValues();
    QVector<double> fingerJointValues();
    QVector<double> leftArmIk();
    QVector<double> rightArmIk();
    QVector<bool> torque();
    ~JointSettingsDialog();

public slots:
    void setJointsFromIK(QVector<double> joints, QVector<double> hands);

private slots:
    void playMotion();

signals:
    void updateMotion(QVector<double>,QVector<double>,QVector<double>);
    void btnIkRequest(QVector<double>);
    void torqueRequest();
    void ikRequest();
    void jointValueChanged();
    void leftIkRequest();
    void rightIkRequest();

private:
    Ui::JointSettingsDialog *ui;
    QVector<double> left_arm;
    QVector<double> right_arm;
    QVector<double> left_finger;
    QVector<double> right_finger;
    QVector<QSlider*> arm_slider;
    QVector<QSlider*> finger_slider;
    QVector<double> motion_ik0;
    QVector<double> motion_ik1;
    QVector<double> time;
    QTimer *motion_timer;
};

#endif // JOINTSETTINGSDIALOG_H
