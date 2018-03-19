#include "widget3d.h"
#include "scenemodifier.h"
#include <QtWidgets>

Widget3D::Widget3D()
{
    init();
}

QWidget *Widget3D::getContainer()
{
    return container;
}

inline
void Widget3D::init()
{
    view->defaultFrameGraph()->setClearColor(QColor(Qt::black));
    auto screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200,100));
//    container->setMaximumSize(screenSize);
    view->registerAspect(input);

    cameraEntity->lens()->setPerspectiveProjection(45.0f,16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(0, 0, 20.0f));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 0));

    light->setColor("white");
    light->setIntensity(0.5);
    lightEntity->addComponent(light);
    lightTransform->setTranslation(cameraEntity->position()-QVector3D(0.0,0.0,-40.0));
    lightEntity->addComponent(lightTransform);

    light2->setColor("white");
    light2->setIntensity(0.5);
    lightEntity2->addComponent(light2);
    lightTransform2->setTranslation(cameraEntity->position()-QVector3D(0.0,0.0,40.0));
    lightEntity2->addComponent(lightTransform2);

//    light3->setColor("white");
//    light3->setIntensity(0.5);
//    lightEntity3->addComponent(light3);
//    lightTransform3->setTranslation(QVector3D(-40.0,0.0,0.0));
//    lightEntity3->addComponent(lightTransform3);

//    light4->setColor("white");
//    light4->setIntensity(0.5);
//    lightEntity4->addComponent(light4);
//    lightTransform4->setTranslation(QVector3D(40.0,0.0,0.0));
//    lightEntity4->addComponent(lightTransform4);

//    light5->setColor("white");
//    light5->setIntensity(0.5);
//    lightEntity5->addComponent(light5);
//    lightTransform5->setTranslation(QVector3D(0.0,40.0,0.0));
//    lightEntity5->addComponent(lightTransform5);

//    camController->setCamera(cameraEntity);
    orbitalCamController->setCamera(cameraEntity);
    orbitalCamController->setLookSpeed(2000.0);
    orbitalCamController->setLinearSpeed(10.0);

    modifier = new SceneModifier(rootEntity);
    view->setRootEntity(rootEntity);
}
