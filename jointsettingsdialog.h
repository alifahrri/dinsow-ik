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
    ~JointSettingsDialog();

signals:
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
};

#endif // JOINTSETTINGSDIALOG_H
