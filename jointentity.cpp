#include "jointentity.h"

JointEntity::JointEntity(Qt3DCore::QEntity *root, double radius)
{
    mesh = new Qt3DExtras::QSphereMesh();
    mesh->setRadius(radius);

    transform = new Qt3DCore::QTransform();
    transform->setTranslation(QVector3D(0.0f,0.0f,0.0f));

    material = new Qt3DExtras::QPhongAlphaMaterial();
    material->setAlpha(0.1);
    material->setDiffuse(QColor(Qt::red));
    material->setShininess(1.0);

    this->addComponent(mesh);
    this->addComponent(material);
    this->addComponent(transform);
    this->setParent(root);
}
