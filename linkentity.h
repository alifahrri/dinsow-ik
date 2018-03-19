#ifndef LINKENTITY_H
#define LINKENTITY_H

#include <QEntity>
#include <Qt3DRender>
#include <Qt3DExtras>

class LinkEntity : public Qt3DCore::QEntity
{
public:
    //geometry type
    enum LinkType
    {
        SPHERE,
        CUBOID,
        CYLINDER
    };

public:
    LinkEntity(LinkType type, Qt3DCore::QEntity* root, QVector<double> prop = QVector<double>(), QColor color = QColor(Qt::gray), QUrl file = QUrl());
public:
    Qt3DCore::QTransform *transform;
private:
    Qt3DRender::QGeometryRenderer* mesh;
//    Qt3DExtras::QPhongAlphaMaterial *material;
    Qt3DExtras::QPhongMaterial *material;
    LinkType link_type;
};

#endif // LINKENTITY_H
