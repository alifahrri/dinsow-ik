#include "dinsowkinematic.h"

#define DEBUG
#define TEST

#ifdef DEBUG
#include <iostream>
#endif

using namespace KDL;

DinsowKinematic::DinsowKinematic()
{
    lhand_chain.addSegment(Segment(Joint("j0",Joint::None)));
    lhand_chain.addSegment(Segment(Joint("j1",Joint::RotY),Frame(Vector(0.0,10.0,0.0))));
    lhand_chain.addSegment(Segment(Joint("j2",Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    lhand_chain.addSegment(Segment(Joint("j3",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    lhand_chain.addSegment(Segment(Joint("j4",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    lhand_chain.addSegment(Segment(Joint("j5",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    lhand_chain.addSegment(Segment(Joint("j6",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));

    lhand_chain_fk = new ChainFkSolverPos_recursive(lhand_chain);
    lhand_ik_vel = new ChainIkSolverVel_pinv(lhand_chain);
    lhand_ik_pos = new ChainIkSolverPos_NR(lhand_chain,*lhand_chain_fk,*lhand_ik_vel);


#ifdef TEST
    auto test_fk = [&](std::vector<double> j)->std::vector<double>
    {
        if(j.size() != 6)
            return std::vector<double>();
        JointValues<6> joint({j[0], j[1], j[2], j[3], j[4], j[5]});
        auto ef = forwardKinematic(joint);
        std::cout << "[forward kinematics]\n"
                  << joint.str() << "; "
                  << ef.str() << '\n';
        return std::vector<double>({ef.x,ef.y,ef.z,ef.rx,ef.ry,ef.rz});
    };
    auto test_ik = [&](std::vector<double> p, std::vector<double> q)->std::vector<double>
    {
        if(p.size()!=6)
            return std::vector<double>();
        if(q.size()!=6)
            return std::vector<double>();
        Pose pose({p.at(0),p.at(1),p.at(2),
                p.at(3),p.at(4),p.at(5)});
        JointValues<6> init_q({q[0],q[1],q[2],
                               q[3],q[4],q[5]});
        auto joints = inverseKinematic(pose,init_q);
        std::cout << "[inverse kinematics]\n"
                  << pose.str() << "; "
                  << joints.str() << '\n';
        return std::vector<double>({joints.q[0], joints.q[1], joints.q[2],
                                    joints.q[3], joints.q[4], joints.q[5]});
    };

    auto j0 = test_fk({0.0,0.0,0.0,0.0,0.0,0.0});
    auto p0 = test_ik(j0,{0.0,0.0,0.0,0.0,0.0,0.0});
    auto j1 = test_fk({1.54,0.0,0.0,0.0,0.0,0.0});
    auto p1 = test_ik(j1,{0.0,0.0,0.0,0.0,0.0,0.0});
#endif
}

DinsowKinematic::Pose DinsowKinematic::forwardKinematic(const DinsowKinematic::ArmJoints &q, DinsowKinematic::ArmSelect_t arm)
{
    Pose p;
    switch (arm) {
    case LEFT:
        left_arm.joint = q;
        p = forwardKinematic(q);
        break;
    case RIGHT:
        right_arm.joint = q;
        p = forwardKinematic(q);
        break;
    default:
        break;
    }
    return p;
}

DinsowKinematic::ArmJoints DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, DinsowKinematic::ArmSelect_t arm)
{
    ArmJoints j;
    switch (arm) {
    case LEFT:
        j = inverseKinematic(p,left_arm.joint);
        break;
    case RIGHT:
        j = inverseKinematic(p,right_arm.joint);
        break;
    }
    return j;
}

DinsowKinematic::ArmJoints DinsowKinematic::joints(DinsowKinematic::ArmSelect_t arm)
{
    switch (arm) {
    case LEFT:
        return left_arm.joint;
    case  RIGHT:
        return right_arm.joint;
    }
}

DinsowKinematic::JointValues<6> DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, const DinsowKinematic::JointValues<6> &init_q)
{
    DinsowKinematic::JointValues<6> joints;
    KDL::JntArray out_q(6);
    KDL::JntArray in_q(lhand_chain.getNrOfJoints());
    for(int i=0; i<6; i++)
        in_q(i) = init_q.q[i];
    KDL::Frame in_frame(Vector(p.x,p.y,p.z));
    lhand_ik_pos->CartToJnt(in_q,in_frame,out_q);
    for(int i=0; i<6; i++)
        joints.q[i] = out_q(i);
    return joints;
}

DinsowKinematic::Pose DinsowKinematic::forwardKinematic(const JointValues<6> &q)
{
    DinsowKinematic::Pose end_ef;
    KDL::JntArray joints(6);
    KDL::Frame in_frame;
    for(int i=0; i<6; i++)
        joints(i) = q.q[i];
    auto err = lhand_chain_fk->JntToCart(joints,in_frame);

    end_ef.x = in_frame.p.x();
    end_ef.y = in_frame.p.y();
    end_ef.z = in_frame.p.z();
    in_frame.M.GetRPY(end_ef.rx,end_ef.ry,end_ef.rz);
    return end_ef;
}

std::__cxx11::string DinsowKinematic::Pose::str()
{
    std::stringstream p;
    p << "pose : ";
    p << "(" << x << "," << y << "," << z << ")";
    p << " ";
    p << "(" << rx << "," << ry << "," << rz << ")";
    return p.str();
}

template<int n>
std::__cxx11::string DinsowKinematic::JointValues<n>::str()
{
    std::stringstream s;
    s << "joints : (";
    for(int i=0; i<n-1; i++)
        s << q[i] << ",";
    s << q[n-1] << ")";
    return s.str();
}
