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
class DinsowMotion;

class SceneModifier : public QObject
{
  Q_OBJECT
public:
  explicit SceneModifier(Qt3DCore::QEntity *_rootEntity);
  ~SceneModifier() {}
  void setTarget(double x, double y, double z);
  void applyFK(QVector<double> q, QVector<double> q_hand = QVector<double>());
  void setDinsow(DinsowKinematic *kinematic, DinsowMotion *motion);
  void dinsowIK(QVector<double> frame);
  void dinsowIKTarget(QVector<double> frame);
  void dinsowMotionIK(QVector<double> f0, QVector<double> f1, QVector<double> t);

private:
  void dinsowFK();
  void setDinsowFK(QVector<double> left_arm, QVector<double> right_arm);
  void initScene();
  void moveLink();
  QQuaternion angle(double q, int idx);
  QQuaternion handAngle(double q, int idx);

signals:
  void jointUpdateIK(QVector<double> joints, QVector<double> hands);

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
  DinsowMotion *motion;
  MeshEntity *dinsow_body;
  MeshEntity *dinsow_head;
  MeshEntity *dinsow_base;
  LinkEntity* body;
  JointEntity *target;
};

#endif // SCENEMODIFIER_H
