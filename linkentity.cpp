#include "linkentity.h"
#include <QDebug>

LinkEntity::LinkEntity(LinkType type, Qt3DCore::QEntity *root, QVector<double> prop, QColor color, QUrl file)
    : Qt3DCore::QEntity(root),
      link_type(type)
{
    if(file.isEmpty())
    {
        switch (type) {
        case CUBOID:
        {
            auto cuboid_mesh = new Qt3DExtras::QCuboidMesh();
            if(prop.size()==3)
            {
                cuboid_mesh->setXExtent(prop.at(0));
                cuboid_mesh->setYExtent(prop.at(1));
                cuboid_mesh->setZExtent(prop.at(2));
            }
            else
            {
                cuboid_mesh->setXExtent(1.0f);
                cuboid_mesh->setYExtent(1.0f);
                cuboid_mesh->setZExtent(1.0f);
            }
            mesh = cuboid_mesh;
            break;
        }
        case SPHERE:
        {
            auto sphere_mesh = new Qt3DExtras::QSphereMesh();
            if(prop.size()==1)
                sphere_mesh->setRadius(prop.at(0));
            else
                sphere_mesh->setRadius(1.0f);
            sphere_mesh->setSlices(10);
            sphere_mesh->setRings(10);
            mesh = sphere_mesh;
            break;
        }
        case CYLINDER:
        {
            auto cylinder_mesh = new Qt3DExtras::QCylinderMesh();
            if(prop.size()==2)
            {
                cylinder_mesh->setRadius(prop.at(0));
                cylinder_mesh->setLength(prop.at(1));
            }
            else
            {
                cylinder_mesh->setRadius(1.0f);
                cylinder_mesh->setLength(3.0f);
            }
            cylinder_mesh->setSlices(50);
            cylinder_mesh->setRings(50);
            mesh = cylinder_mesh;
            break;
        }
        default:
            break;
        }
        transform = new Qt3DCore::QTransform();
    //    transform->setTranslation(QVector3D(0.0f,0.0f,0.0f));

    //    material = new Qt3DExtras::QPhongAlphaMaterial();
        material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(color);
    //    material->setShininess(1.0);
        material->setSpecular(color);

        this->addComponent(mesh);
        this->addComponent(material);
    }
    else
    {
//        auto mesh_file = new Qt3DRender::QMesh();
//        mesh_file->setSource(file);
//        mesh = mesh_file;
        qDebug() << "setting mesh" << file.fileName();
        mesh = new Qt3DRender::QMesh(root);
        dynamic_cast<Qt3DRender::QMesh*>(mesh)->setSource(file);
        qDebug() << "mesh source :" << static_cast<Qt3DRender::QMesh*>(mesh)->source().toString();
        transform = new Qt3DCore::QTransform();
    //    transform->setTranslation(QVector3D(0.0f,0.0f,0.0f));

    //    material = new Qt3DExtras::QPhongAlphaMaterial();
        material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(color);
    //    material->setShininess(1.0);
        material->setSpecular(color);

        this->addComponent(mesh);
        this->addComponent(material);
    }
    this->addComponent(transform);
    this->setParent(root);
}
