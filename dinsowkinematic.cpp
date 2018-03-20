#include "dinsowkinematic.h"

#define DEBUG
//#define TEST_CHAIN
#define USE_ROTATION
#define USE_LMA

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

    left_arm.chain.addSegment(Segment(Joint("j0",Joint::None)));
    left_arm.chain.addSegment(Segment(Joint("j1",Joint::RotY),Frame(Vector(0.0,10.0,0.0))));
    left_arm.chain.addSegment(Segment(Joint("j2",Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    left_arm.chain.addSegment(Segment(Joint("j3",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    left_arm.chain.addSegment(Segment(Joint("j4",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    left_arm.chain.addSegment(Segment(Joint("j5",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    left_arm.chain.addSegment(Segment(Joint("j6",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));

    Eigen::Matrix<double,6,1> w_mat;
    std::cout << "w : " << w_mat << '\n';
    w_mat(0,0) = 1.0;
    w_mat(1,0) = 1.0;
    w_mat(2,0) = 1.0;

    w_mat(3,0) = 1.0;
    w_mat(4,0) = 1.0;
    w_mat(5,0) = 1.0;
    std::cout << "w : " << w_mat << '\n';

    left_arm.fk_solver = new ChainFkSolverPos_recursive(left_arm.chain);
    left_arm.ikvel_solver = new ChainIkSolverVel_pinv(left_arm.chain);
    left_arm.ikvel_solver_wdls = new ChainIkSolverVel_wdls(left_arm.chain);
    left_arm.ikpos_solver_lma = new ChainIkSolverPos_LMA(left_arm.chain,w_mat);
    left_arm.ikpos_solver = new ChainIkSolverPos_NR(left_arm.chain,*(left_arm.fk_solver),*(left_arm.ikvel_solver_wdls),500);

    JntArray q_min(7);
    JntArray q_max(7);

    q_min(0) = 0.0;     q_max(0) = 0.0;
    q_min(1) = M_PI;    q_max(1) = -M_PI;
    q_min(2) = M_PI;    q_max(2) = -M_PI;
    q_min(3) = M_PI;    q_max(3) = -M_PI;
    q_min(4) = M_PI;    q_max(4) = -M_PI;
    q_min(5) = M_PI;    q_max(5) = -M_PI;
    q_min(6) = M_PI;    q_max(6) = -M_PI;
    left_arm.ikpos_solver_jl = new ChainIkSolverPos_NR_JL(left_arm.chain,q_min,q_max,*(left_arm.fk_solver),*(left_arm.ikvel_solver),5000);

    right_arm.chain.addSegment(Segment(Joint("j0",Joint::None)));
    right_arm.chain.addSegment(Segment(Joint("j1",Joint::RotY),Frame(Vector(0.0,-10.0,0.0))));
    right_arm.chain.addSegment(Segment(Joint("j2",Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    right_arm.chain.addSegment(Segment(Joint("j3",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    right_arm.chain.addSegment(Segment(Joint("j4",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    right_arm.chain.addSegment(Segment(Joint("j5",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    right_arm.chain.addSegment(Segment(Joint("j6",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));

    right_arm.fk_solver = new ChainFkSolverPos_recursive(right_arm.chain);
    right_arm.ikvel_solver = new ChainIkSolverVel_pinv(right_arm.chain);
    right_arm.ikvel_solver_wdls = new ChainIkSolverVel_wdls(right_arm.chain);
    right_arm.ikpos_solver_lma = new ChainIkSolverPos_LMA(right_arm.chain,w_mat);
    right_arm.ikpos_solver = new ChainIkSolverPos_NR(right_arm.chain,*(right_arm.fk_solver),*(right_arm.ikvel_solver));

    q_min(0) = 0.0;     q_max(0) = 0.0;
    q_min(1) = M_PI;    q_max(1) = -M_PI;
    q_min(2) = M_PI;    q_max(2) = -M_PI;
    q_min(3) = M_PI;    q_max(3) = -M_PI;
    q_min(4) = M_PI;    q_max(4) = -M_PI;
    q_min(5) = M_PI;    q_max(5) = -M_PI;
    q_min(6) = M_PI;    q_max(6) = -M_PI;
    right_arm.ikpos_solver_jl = new ChainIkSolverPos_NR_JL(right_arm.chain,q_min,q_max,*(right_arm.fk_solver),*(right_arm.ikvel_solver));

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

#ifdef TEST_CHAIN
    auto test_fk = [&](std::vector<double> j, ArmSelect_t arm)->std::vector<double>
    {
        if(j.size() != 6)
            return std::vector<double>();
        JointValues<6> joint({j[0], j[1], j[2], j[3], j[4], j[5]});
        auto ef = forwardKinematic(joint, arm);
        std::cout << "[forward kinematics] " << (arm==LEFT? "left" : "right") << '\n'
                  << joint.str() << "; "
                  << ef.str() << '\n';
        return std::vector<double>({ef.x,ef.y,ef.z,ef.rx,ef.ry,ef.rz});
    };
    auto test_ik = [&](std::vector<double> p, ArmSelect_t arm)->std::vector<double>
    {
        if(p.size()!=6)
            return std::vector<double>();
        Pose pose({p.at(0),p.at(1),p.at(2),
                p.at(3),p.at(4),p.at(5)});
        auto joints = inverseKinematic(pose,arm);
        std::cout << "[inverse kinematics] " << (arm==LEFT? "left" : "right") << '\n'
                  << pose.str() << "; "
                  << joints.str() << '\n';
        return std::vector<double>({joints.q[0], joints.q[1], joints.q[2],
                                    joints.q[3], joints.q[4], joints.q[5]});
    };

    auto normalize = [](const std::vector<double> &joint)->std::vector<double>
    {
        std::vector<double> ret(joint.size());
        for(size_t i=0; i<joint.size(); i++)
        {
            double angle = joint[i];
            while(angle>M_PI)
                angle -= (2*M_PI);
            while(angle<-M_PI)
                angle += (2*M_PI);
            ret[i] = angle;
        }
        return ret;
    };

    auto v0 = std::vector<double>({0.0,0.0,0.0,0.0,0.0,0.0});
    auto v1 = std::vector<double>({1.54,0.0,0.0,0.0,0.0,0.0});

    std::cout << "TEST IK (1) : \n";
    auto j = test_ik({10.0,10.0,10.0,0.0,0.0,0.0},LEFT);
    auto j_normalized = normalize(j);
    test_fk(j_normalized,LEFT);
    std::cout << "TEST IK (2) : \n";
    auto j1 = test_ik({10.0,-10.0,10.0,0.0,0.0,0.0},RIGHT);
    auto j1_normalized = normalize(j1);
    test_fk(j1_normalized,RIGHT);
    std::cout << "TEST IK (3) : \n";
    auto j2 = test_ik({0.0,5.0,30.0,0.0,0.0,0.0},LEFT);
    auto j2_normalized = normalize(j2);
    test_fk(j2_normalized,LEFT);
#endif
}

DinsowKinematic::Pose DinsowKinematic::forwardKinematic(const DinsowKinematic::ArmJoints &q, DinsowKinematic::ArmSelect_t arm)
{
    Pose p;
    switch (arm) {
    case LEFT:
        p = forwardKinematic(q,left_arm);
        break;
    case RIGHT:
        p = forwardKinematic(q,right_arm);
        break;
    default:
        break;
    }
    return p;
}

DinsowKinematic::ArmJoints DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, DinsowKinematic::ArmSelect_t arm)
{
    std::cout << "[InverseKinematic] p : " << p.str() << std::endl;
    ArmJoints j;
    switch (arm) {
    case LEFT:
        j = inverseKinematic(p,left_arm);
        break;
    case RIGHT:
        j = inverseKinematic(p,right_arm);
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

DinsowKinematic::Pose DinsowKinematic::forwardKinematic(const ArmJoints &q, DinsowArmChain &chain)
{
    DinsowKinematic::Pose end_ef;
    auto n = chain.chain.getNrOfJoints();
    KDL::JntArray joints(n);
    KDL::Frame in_frame;
    for(int i=0; i<n; i++)
        joints(i) = q.q[i];
    auto err = chain.fk_solver->JntToCart(joints,in_frame);

    end_ef.x = in_frame.p.x();
    end_ef.y = in_frame.p.y();
    end_ef.z = in_frame.p.z();
    in_frame.M.GetRPY(end_ef.rx,end_ef.ry,end_ef.rz);

    chain.joint = q;
    return end_ef;
}

DinsowKinematic::ArmJoints DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, DinsowArmChain &chain)
{
    auto n = chain.chain.getNrOfJoints();
    ArmJoints joints;
    KDL::JntArray out_q(n);
    KDL::JntArray in_q(n);
    const auto& init_q = chain.joint;
    for(int i=0; i<n; i++)
        in_q(i) = init_q.q[i];
#ifdef USE_ROTATION
    KDL::Rotation rot = KDL::Rotation::RPY(p.rx,p.ry,p.rz);
    KDL::Frame in_frame(rot,Vector(p.x,p.y,p.z));
#else
    KDL::Frame in_frame(Vector(p.x,p.y,p.z));
#endif
#ifdef USE_LMA
    chain.ikpos_solver_lma->display_information = true;
    auto err = chain.ikpos_solver_lma->CartToJnt(in_q,in_frame,out_q);
    std::cout << "IK : " << (err==0 ? "success\n" : (err==-1 ? "gradients towards joint is too small\n" : (err==-2 ? "joint position increments are too small\n" : "number of iteration is exceeded\n")));
#else
    auto err = chain.ikpos_solver_jl->CartToJnt(in_q,in_frame,out_q);
    std::cout << "IK : " << chain.ikpos_solver->strError(err) << '\n';
#endif
    for(int i=0; i<n; i++)
        joints.q[i] = out_q(i);
#ifndef USE_ROTATION
    joints.q[4] = p.rx;
    joints.q[5] = p.ry;
#endif
    normalize(joints);
    chain.joint = joints;
    return joints;
}

DinsowKinematic::ArmJoints DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, const ArmJoints &init_q)
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

void DinsowKinematic::normalize(DinsowKinematic::ArmJoints &arm_joints)
{
    for(size_t i=0; i<6; i++)
    {
        while(arm_joints.q[i] > M_PI)
            arm_joints.q[i] -= (2*M_PI);
        while(arm_joints.q[i] < (-M_PI))
            arm_joints.q[i] += (2*M_PI);
    }
}

DinsowKinematic::Pose DinsowKinematic::forwardKinematic(const ArmJoints &q)
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

std::__cxx11::string DinsowKinematic::Pose::str() const
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

