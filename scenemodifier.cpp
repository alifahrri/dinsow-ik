#include "scenemodifier.h"
#include "dinsowkinematic.h"

SceneModifier::SceneModifier(Qt3DCore::QEntity *_rootEntity)
    : rootEntity(_rootEntity)
{
    initScene();
    applyFK({0.0,0.0,0.0,0.0,0.0,0.0,0.0,
             0.0,0.0,0.0,0.0,0.0,0.0,0.0});
}

void SceneModifier::dinsowFK()
{
    if(!dinsow)
        return;
}

inline
void SceneModifier::initScene()
{
    link.resize(14);
    joint.resize(14);
    relativePos.resize(14);
    linkRelativePos.resize(14);
    hand_joint.resize(22);
    hand_link.resize(22);
    handRelativePos.resize(22);
    handLinkRelativePos.resize(22);

    relativePos[0] = QVector3D(1.0,2.5,0.0);
    relativePos[1] = QVector3D(1.0,0.0,0.0);
    relativePos[2] = QVector3D(0.0,0.0,0.0);
    relativePos[3] = QVector3D(0.0,-0.5-1.0,0.0);
    relativePos[4] = QVector3D(0.0,-1.0-0.5,0.0);
    relativePos[5] = QVector3D(0.0,-2.0,0.0);
    relativePos[6] = QVector3D(0.0,0.0,0.0);

    relativePos[7] = QVector3D(-1.75,2.5,0.0);
    relativePos[8] = QVector3D(-1.0,0.0,0.0);
    relativePos[9] = QVector3D(0.0,0.0,0.0);
    relativePos[10] = QVector3D(0.0,-0.5-1.0,0.0);
    relativePos[11] = QVector3D(0.0,-1.0-0.5,0.0);
    relativePos[12] = QVector3D(0.0,-2.0,0.0);
    relativePos[13] = QVector3D(0.0,0.0,0.0);

    linkRelativePos[0] = QVector3D(1.0,2.5,0.0);
    linkRelativePos[1] = QVector3D(0.0,0.0,0.0);
    linkRelativePos[2] = QVector3D(0.0,-1.0,0.0);
    linkRelativePos[3] = QVector3D(0.0,-1.0,0.0);
    linkRelativePos[4] = QVector3D(0.0,-1.0,0.0);
    linkRelativePos[5] = QVector3D(0.0,0.0,0.0);
    linkRelativePos[6] = QVector3D(0.0,-0.5,0.0);

    linkRelativePos[7] = QVector3D(-1.75,2.5,0.0);
    linkRelativePos[8] = QVector3D(0.0,0.0,0.0);
    linkRelativePos[9] = QVector3D(0.0,-1.0,0.0);
    linkRelativePos[10] = QVector3D(0.0,-1.0,0.0);
    linkRelativePos[11] = QVector3D(0.0,-1.0,0.0);
    linkRelativePos[12] = QVector3D(0.0,0.0,0.0);
    linkRelativePos[13] = QVector3D(0.0,-0.5,0.0);

    handRelativePos[0] = QVector3D(0.5,-0.7,0.15);
    handRelativePos[1] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[2] = QVector3D(0.3,-1.0,0.0);
    handRelativePos[3] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[4] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[5] = QVector3D(0.0,-1.0,0.0);
    handRelativePos[6] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[7] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[8] = QVector3D(-0.3,-1.0,0.0);
    handRelativePos[9] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[10] = QVector3D(0.0,-0.3,0.0);

    handRelativePos[11] = QVector3D(-0.5,-0.7,0.15);
    handRelativePos[12] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[13] = QVector3D(0.3,-1.0,0.0);
    handRelativePos[14] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[15] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[16] = QVector3D(0.0,-1.0,0.0);
    handRelativePos[17] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[18] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[19] = QVector3D(-0.3,-1.0,0.0);
    handRelativePos[20] = QVector3D(0.0,-0.3,0.0);
    handRelativePos[21] = QVector3D(0.0,-0.3,0.0);

    for(int i=0; i<22; i++)
        handLinkRelativePos[i] = QVector3D(0.0,-0.15,0.0);

    dinsow_body = new MeshEntity(rootEntity,QUrl("qrc:/mesh/body.OBJ"),QColor(Qt::white),0.015);
    dinsow_body->transform->setRotationX(-90.0);
    dinsow_body->transform->setTranslation(QVector3D(-3.55,-9.7,3.1275));

    dinsow_head = new MeshEntity(rootEntity,QUrl("qrc:/mesh/head.OBJ"),QColor(Qt::white),0.015);
    dinsow_head->transform->setRotationX(-90.0);
    dinsow_head->transform->setTranslation(QVector3D(-2.55,-0.5,1.5));

    dinsow_base = new MeshEntity(rootEntity,QUrl("qrc:/mesh/dinsow_base.OBJ"),QColor(Qt::white),0.015);
//    dinsow_base->transform->setRotationX(0.0);
    dinsow_base->transform->setTranslation(QVector3D(-0.3,-9.25,0.15));

    joint[0] = new JointEntity(rootEntity);
    joint[0]->transform->setTranslation(relativePos[0]);

    joint[7] = new JointEntity(rootEntity);
    joint[7]->transform->setTranslation(relativePos[7]);

    {
        Qt3DCore::QTransform tf_joint[14];
        tf_joint[0].setMatrix((joint[0]->transform)->matrix());

        for(int i=1; i<7; i++)
        {
            QMatrix4x4 mat;
            mat.translate(relativePos[i]);
            mat.rotate(angle(0,i));
            tf_joint[i].setMatrix(tf_joint[i-1].matrix()*mat);
            joint[i] = new JointEntity(rootEntity);
            joint[i]->transform->setMatrix(tf_joint[i].matrix());
        }

        tf_joint[7].setMatrix((joint[7]->transform)->matrix());

        for(int i=8; i<14; i++)
        {
            QMatrix4x4 mat;
            mat.translate(relativePos[i]);
            mat.rotate(angle(0,i));
            tf_joint[i].setMatrix(tf_joint[i-1].matrix()*mat);
            joint[i] = new JointEntity(rootEntity);
            joint[i]->transform->setMatrix(tf_joint[i].matrix());
        }
    }

    {
        Qt3DCore::QTransform tf_joint[22];
        for(int i=0; i<22; i++)
        {
            hand_joint[i] = new JointEntity(rootEntity,0.1);
        }

        for(int i=0; i<22; i++)
        {
            QMatrix4x4 mat;
            mat.translate(handRelativePos[i]);
            if(i==0 || i==2 || i==5 || i==8)
            {
                tf_joint[i].setMatrix(joint.at(6)->transform->matrix()*mat);
                hand_joint[i]->transform->setMatrix(tf_joint[i].matrix());
            }
            else if(i==11 || i==13 || i==16 || i==19)
            {
                tf_joint[i].setMatrix(joint.back()->transform->matrix()*mat);
                hand_joint[i]->transform->setMatrix(tf_joint[i].matrix());
            }
            else
            {
                tf_joint[i].setMatrix(hand_joint[i-1]->transform->matrix()*mat);
                hand_joint[i]->transform->setMatrix(tf_joint[i].matrix());
            }
        }

//        tf_joint[0].setMatrix(joint.back()->transform->matrix()*mat[0]);
//        hand_joint[0]->transform->setMatrix(tf_joint[0].matrix());
//        tf_joint[1].setMatrix(hand_joint[0]->transform->matrix()*mat[1]);
//        hand_joint[1]->transform->setMatrix(tf_joint[1].matrix());

//        tf_joint[2].setMatrix(joint.back()->transform->matrix()*mat[2]);
//        hand_joint[2]->transform->setMatrix(tf_joint[2].matrix());
//        tf_joint[3].setMatrix(hand_joint[2]->transform->matrix()*mat[3]);
//        hand_joint[3]->transform->setMatrix(tf_joint[3].matrix());
//        tf_joint[4].setMatrix(hand_joint[3]->transform->matrix()*mat[4]);
//        hand_joint[4]->transform->setMatrix(tf_joint[4].matrix());

//        tf_joint[5].setMatrix(joint.back()->transform->matrix()*mat[5]);
//        hand_joint[5]->transform->setMatrix(tf_joint[5].matrix());
//        tf_joint[6].setMatrix(hand_joint[5]->transform->matrix()*mat[6]);
//        hand_joint[6]->transform->setMatrix(tf_joint[6].matrix());
//        tf_joint[7].setMatrix(hand_joint[6]->transform->matrix()*mat[7]);
//        hand_joint[7]->transform->setMatrix(tf_joint[7].matrix());

//        tf_joint[8].setMatrix(joint.back()->transform->matrix()*mat[8]);
//        hand_joint[8]->transform->setMatrix(tf_joint[8].matrix());
//        tf_joint[9].setMatrix(hand_joint[8]->transform->matrix()*mat[9]);
//        hand_joint[9]->transform->setMatrix(tf_joint[9].matrix());
//        tf_joint[10].setMatrix(hand_joint[9]->transform->matrix()*mat[10]);
//        hand_joint[10]->transform->setMatrix(tf_joint[10].matrix());
    }

    link[0] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,1.5});
    link[0]->transform->setTranslation(linkRelativePos[0]);
    link[0]->transform->setRotationZ(90.0);
    link[1] = new LinkEntity(LinkEntity::SPHERE,rootEntity,{0.5},QColor(Qt::green));
    link[2] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,2.0});
    link[3] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,1.0},QColor(Qt::green));
    link[4] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,2.0},QColor(255,255,255,127));
    link[5] = new LinkEntity(LinkEntity::SPHERE,rootEntity,{0.5},QColor(Qt::green));
    link[6] = new LinkEntity(LinkEntity::CUBOID,rootEntity,{0.75,1.0,0.25},QColor(Qt::green));

    link[7] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,1.5});
    link[7]->transform->setTranslation(linkRelativePos[7]);
    link[7]->transform->setRotationZ(90.0);
    link[8] = new LinkEntity(LinkEntity::SPHERE,rootEntity,{0.5},QColor(Qt::green));
    link[9] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,2.0});
    link[10] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,1.0},QColor(Qt::green));
    link[11] = new LinkEntity(LinkEntity::CYLINDER,rootEntity,{0.5,2.0},QColor(255,255,255,127));
    link[12] = new LinkEntity(LinkEntity::SPHERE,rootEntity,{0.5},QColor(Qt::green));
    link[13] = new LinkEntity(LinkEntity::CUBOID,rootEntity,{0.75,1.0,0.25},QColor(Qt::green));

    for(int i=0; i<22; i++)
        hand_link[i] = new LinkEntity(LinkEntity::CUBOID,rootEntity,{0.1,0.3,0.1});

    applyFK({0.0,0.0,0.0,0.0,0.0,0.0,0.0});

//    auto tf = joint[1]->transform->translation();
//    tf += linkRelativePos[1];
//    link[1]->transform->setTranslation(tf);

//    auto tf2 = joint[2]->transform->translation();
//    tf2 += linkRelativePos[2];
//    link[2]->transform->setTranslation(tf2);

//    auto tf3 = joint[3]->transform->translation();
//    tf3 += linkRelativePos[3];
//    link[3]->transform->setTranslation(tf3);

//    auto tf4 = joint[4]->transform->translation();
//    tf4 += linkRelativePos[4];
//    link[4]->transform->setTranslation(tf4);

//    auto tf5 = joint[5]->transform->translation();
//    tf5 += linkRelativePos[5];
//    link[5]->transform->setTranslation(tf5);

}

void SceneModifier::moveLink()
{
    for(int i=1; i<7; i++)
    {
//        auto mat = joint[i]->transform->matrix();
//        tf += linkRelativePos[i];
        QMatrix4x4 mat;
        mat.translate(linkRelativePos[i]);
        link[i]->transform->setMatrix(joint[i]->transform->matrix()*mat);
    }

    for(int i=8; i<14; i++)
    {
        QMatrix4x4 mat;
        mat.translate(linkRelativePos[i]);
        link[i]->transform->setMatrix(joint[i]->transform->matrix()*mat);
    }

    for(int i=0; i<22; i++)
    {
        QMatrix4x4 mat;
        mat.translate(handLinkRelativePos[i]);
        hand_link[i]->transform->setMatrix(hand_joint[i]->transform->matrix()*mat);
    }
}

void SceneModifier::applyFK(QVector<double> q, QVector<double> q_hand)
{
    if(q.size()!=14)
        return;
    Qt3DCore::QTransform tf[14];
    Qt3DCore::QTransform tf_joint[22];
//    tf[0].setMatrix((link[0]->transform)->matrix());
    tf[0].setMatrix((joint[0]->transform)->matrix());
    tf[0].setRotationZ(0.0);
    tf[7].setMatrix((joint[7]->transform)->matrix());
    tf[7].setRotationZ(0.0);

    for(int i=1; i<7; i++)
    {
        QMatrix4x4 mat;
        mat.translate(relativePos[i]);
        mat.rotate(angle(q[i],i));
        tf[i].setMatrix(tf[i-1].matrix()*mat);
        joint[i]->transform->setMatrix(tf[i].matrix());
    }

    for(int i=8; i<14; i++)
    {
        QMatrix4x4 mat;
        mat.translate(relativePos[i]);
        mat.rotate(angle(q[i],i));
        tf[i].setMatrix(tf[i-1].matrix()*mat);
        joint[i]->transform->setMatrix(tf[i].matrix());
    }

    if(q_hand.size()!=22)
        goto MOVE_LINK;
    for(int i=0; i<22; i++)
    {
        QMatrix4x4 mat;
        mat.translate(handRelativePos[i]);
        mat.rotate(handAngle(q_hand[i],i));
        if(i==0 || i==2 || i==5 || i==8)
        {
//            tf_joint[i].setMatrix(joint.back()->transform->matrix()*mat);
            tf_joint[i].setMatrix(joint.at(6)->transform->matrix()*mat);
            hand_joint[i]->transform->setMatrix(tf_joint[i].matrix());
        }
        else if(i==11 || i==13 || i==16 || i==19)
        {
            tf_joint[i].setMatrix(joint.back()->transform->matrix()*mat);
            hand_joint[i]->transform->setMatrix(tf_joint[i].matrix());
        }
        else
        {
            tf_joint[i].setMatrix(hand_joint[i-1]->transform->matrix()*mat);
            hand_joint[i]->transform->setMatrix(tf_joint[i].matrix());
        }
    }
    MOVE_LINK:
    moveLink();
}

QQuaternion SceneModifier::angle(double q, int idx)
{
    QQuaternion qq;
    qq = QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),1);
    switch (idx) {
    case 1:
    case 4:
    case 6:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),q);
        break;
    case 2:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(0,0,1),q);
        break;
    case 3:
    case 5:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(0,1,0),q);
        break;

    case 8:
    case 11:
    case 13:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),q);
        break;
    case 9:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(0,0,1),q);
        break;
    case 10:
    case 12:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(0,1,0),q);
        break;
    default:
        break;
    }
    return qq;
}

QQuaternion SceneModifier::handAngle(double q, int idx)
{
    QQuaternion qq;
    qq = QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),1);
    switch (idx) {
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:

    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),q);
        break;
    case 0:
    case 1:
    case 11:
    case 12:
        qq = QQuaternion::fromAxisAndAngle(QVector3D(0,0,1),q);
        break;
    default:
        break;
    }
    return qq;
}
