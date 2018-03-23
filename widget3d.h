#ifndef WIDGET3D_H
#define WIDGET3D_H

#include <QWidget>

#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcameralens.h>

#include <Qt3DInput/qinputaspect.h>

#include <Qt3DExtras/qtorusmesh.h>
#include <Qt3DRender/qmesh.h>
#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qsceneloader.h>
#include <Qt3DRender/qpointlight.h>

#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qfirstpersoncameracontroller.h>
#include <Qt3DExtras/QOrbitCameraController>

class SceneModifier;
class DinsowKinematic;
class DinsowMotion;

class Widget3D
{
public:
    Widget3D();
    QWidget *getContainer();
    void passDinsow(DinsowKinematic *dinsow, DinsowMotion *motion);
public:
    SceneModifier *modifier;
private:
    void init();
private:
    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    QWidget* container = QWidget::createWindowContainer(view);
    Qt3DInput::QInputAspect *input = new Qt3DInput::QInputAspect;
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();
    Qt3DRender::QCamera *cameraEntity = view->camera();

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);

    Qt3DCore::QEntity *lightEntity2 = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light2 = new Qt3DRender::QPointLight(lightEntity2);
    Qt3DCore::QTransform *lightTransform2 = new Qt3DCore::QTransform(lightEntity2);

    Qt3DExtras::QFirstPersonCameraController *camController = new Qt3DExtras::QFirstPersonCameraController(rootEntity);
    Qt3DExtras::QOrbitCameraController *orbitalCamController = new Qt3DExtras::QOrbitCameraController(rootEntity);
};

#endif // WIDGET3D_H
