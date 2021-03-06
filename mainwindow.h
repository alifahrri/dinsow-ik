#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <widget3d.h>
#include <jointsettingsdialog.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    Widget3D *getWidget3D();
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public:
    JointSettingsDialog *joint_dialog;
    Widget3D *widget3d;
};

#endif // MAINWINDOW_H
