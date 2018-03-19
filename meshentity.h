#ifndef MESHENTITY_H
#define MESHENTITY_H

#include <Qt3DRender>
#include <Qt3DExtras>

class MeshEntity : public Qt3DCore::QEntity
{
public:
    MeshEntity(Qt3DCore::QEntity *root, QUrl file, QColor color = QColor(Qt::white), double factor=1.0);
public:
    Qt3DCore::QTransform *transform;
private:
    Qt3DRender::QMesh *mesh;
    Qt3DExtras::QPhongMaterial *material;
};

#endif // MESHENTITY_H
