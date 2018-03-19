#include "meshentity.h"
#include <QDebug>

MeshEntity::MeshEntity(Qt3DCore::QEntity *root, QUrl file, QColor color, double factor)
{
    transform = new Qt3DCore::QTransform();
    transform->setScale(factor);
    mesh = new Qt3DRender::QMesh();
    mesh->setMeshName("dinsowbody");
    mesh->setSource(file);
    material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(color);
    qDebug() << "mesh : " << mesh->source().toString();
    this->addComponent(mesh);
    this->addComponent(transform);
    this->addComponent(material);
    this->setParent(root);
}
