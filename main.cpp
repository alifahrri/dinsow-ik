#include "mainwindow.h"
#include <QApplication>
#include <dinsowkinematic.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Q_INIT_RESOURCE(dinsowbody);
    DinsowKinematic dinsow_kinematic;
    MainWindow w;
    w.showMaximized();

    return a.exec();
}
