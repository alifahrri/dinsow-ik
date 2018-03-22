#include <QApplication>
#include "mainwindow.h"
#include "dinsowkinematic.h"
#include "dinsowmotion.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(dinsowbody);
    DinsowKinematic dinsow_kinematic;
    DinsowMotion dinsow_motion(dinsow_kinematic);
    MainWindow w;
    w.getWidget3D()->passKinematic(&dinsow_kinematic);
    w.showMaximized();

    return a.exec();
}
