#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include <QtCore/QObject>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPhongMaterial>

#include "linkentity.h"
#include "jointentity.h"
#include "meshentity.h"

class DinsowKinematic;

class SceneModifier : public QObject
{
    Q_OBJECT
public:
    explicit SceneModifier(Qt3DCore::QEntity *_rootEntity);
    ~SceneModifier() {}
    void applyFK(QVector<double> q, QVector<double> q_hand = QVector<double>());
    void setDinsow(DinsowKinematic *kinematic);
    void dinsowIK(QVector<double> frame);

private:
    void dinsowFK();
    void setDinsowFK(QVector<double> left_arm, QVector<double> right_arm);
    void initScene();
    void moveLink();
    QQuaternion angle(double q, int idx);
    QQuaternion handAngle(double q, int idx);

private:
    Qt3DCore::QEntity *rootEntity;
    QVector<QVector3D> relativePos;
    QVector<QVector3D> linkRelativePos;
    QVector<QVector3D> handRelativePos;
    QVector<QVector3D> handLinkRelativePos;
    QVector<LinkEntity*> link;
    QVector<JointEntity*> joint;
    QVector<JointEntity*> hand_joint;
    QVector<LinkEntity*> hand_link;
    DinsowKinematic *dinsow;
    MeshEntity *dinsow_body;
    MeshEntity *dinsow_head;
    MeshEntity *dinsow_base;
    LinkEntity* body;
};

#endif // SCENEMODIFIER_H
