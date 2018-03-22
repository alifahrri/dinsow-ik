#include "scenemodifier.h"
#include "dinsowkinematic.h"

#define TO_DEG (180.0/M_PI)
#define TO_RAD (M_PI/180.0)
//#define TEST

SceneModifier::SceneModifier(Qt3DCore::QEntity *_rootEntity)
    : rootEntity(_rootEntity),
      dinsow(nullptr)
{
    initScene();
    applyFK({0.0,0.0,0.0,0.0,0.0,0.0,0.0,
             0.0,0.0,0.0,0.0,0.0,0.0,0.0});
}

void SceneModifier::dinsowFK()
{
    if(!dinsow)
        return;
    auto l = dinsow->joints(DinsowKinematic::LEFT);
    auto r = dinsow->joints(DinsowKinematic::RIGHT);
    QVector<double> joints({0.0,l.q[0],l.q[1],l.q[2],l.q[3],l.q[4],l.q[5],
                            0.0,r.q[0],r.q[1],r.q[2],r.q[3],r.q[4],r.q[5]});
    for(auto& j : joints)
        j *= TO_DEG;
    QVector<double> finger_joints(22,0.0);
    applyFK(joints,finger_joints);
}

void SceneModifier::setDinsowFK(QVector<double> left_arm, QVector<double> right_arm)
{
    DinsowKinematic::ArmJoints joints;
    for(int i=0; i<left_arm.size(); i++)
        joints.q[i] = left_arm.at(i);
    dinsow->forwardKinematic(joints,DinsowKinematic::LEFT);
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

    //left joint
    relativePos[0] = QVector3D(1.0,2.5,0.0);
    relativePos[1] = QVector3D(1.0,0.0,0.0);
    relativePos[2] = QVector3D(0.0,0.0,0.0);
    relativePos[3] = QVector3D(0.0,-0.5-1.0,0.0);
    relativePos[4] = QVector3D(0.0,-1.0-0.5,0.0);
    relativePos[5] = QVector3D(0.0,-2.0,0.0);
    relativePos[6] = QVector3D(0.0,0.0,0.0);

    //right joint
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
}

void SceneModifier::moveLink()
{
    for(int i=1; i<7; i++)
    {
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

void SceneModifier::setDinsow(DinsowKinematic *kinematic)
{
    dinsow = kinematic;
#ifdef TEST
    dinsowIK(QVector<double>({0.0,10.0,-20.0,0.0,0.0,0.0}));
#endif
}

void SceneModifier::dinsowIK(QVector<double> frame)
{
    if(!dinsow)
        return;
    auto left_pose = DinsowKinematic::Pose({frame[0],frame[1],frame[2],
                                       frame[3],frame[4],frame[5]});
    auto right_pose = DinsowKinematic::Pose({frame[6],frame[7],frame[8],
                                       frame[9],frame[10],frame[11]});
    auto left_joints = dinsow->inverseKinematic(left_pose,DinsowKinematic::LEFT);
    auto right_joints = dinsow->inverseKinematic(right_pose,DinsowKinematic::RIGHT);
    QVector<double> qjoints, qhands;
    qjoints.push_back(0.0);
    for(size_t i=0; i<6; i++)
        qjoints.push_back(left_joints.q[i]*TO_DEG);
    qjoints.push_back(0.0);
    for(size_t i=0; i<6; i++)
        qjoints.push_back(right_joints.q[i]*TO_DEG);
    for(size_t i=0; i<22; i++)
        qhands.push_back(0.0);
    applyFK(qjoints,qhands);
    emit jointUpdateIK(qjoints,qhands);
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
