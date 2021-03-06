#ifndef JOINTENTITY_H
#define JOINTENTITY_H

#include <QEntity>
#include <Qt3DRender>
#include <Qt3DExtras>

class JointEntity : public Qt3DCore::QEntity
{
public:
    JointEntity(Qt3DCore::QEntity *root, double alpha = 0.1, double radius = 0.15);
public:
    Qt3DCore::QTransform *transform;
private:
    Qt3DExtras::QSphereMesh *mesh;
    Qt3DExtras::QPhongAlphaMaterial *material;
    //    Qt3DRender::QMaterial *material;
};

#endif // JOINTENTITY_H
