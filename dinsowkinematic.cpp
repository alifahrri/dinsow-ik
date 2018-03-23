#include "dinsowkinematic.h"

#define DEBUG
//#define TEST_CHAIN
//#define DEBUG_IK
//#define USE_ROTATION
#define USE_LMA

#ifdef DEBUG
#include <iostream>
#endif

using namespace KDL;

DinsowKinematic::DinsowKinematic()
{
    left_arm.chain.addSegment(Segment(Joint("j0",Joint::None)));
    left_arm.chain.addSegment(Segment(Joint("j1",Joint::RotY),Frame(Vector(0.0,10.0,0.0))));
    left_arm.chain.addSegment(Segment(Joint("j2",Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    left_arm.chain.addSegment(Segment(Joint("j3",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    left_arm.chain.addSegment(Segment(Joint("j4",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    left_arm.chain.addSegment(Segment(Joint("j5",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    left_arm.chain.addSegment(Segment(Joint("j6",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));

    Eigen::Matrix<double,6,1> w_mat;
    w_mat(0,0) = 1.0;
    w_mat(1,0) = 1.0;
    w_mat(2,0) = 1.0;

#ifdef USE_ROTATION
    w_mat(3,0) = 0.6;
    w_mat(4,0) = 0.6;
    w_mat(5,0) = 0.6;
#else
    w_mat(3,0) = 0.025;
    w_mat(4,0) = 0.025;
    w_mat(5,0) = 0.025;
#endif
#ifdef DEBUG_IK
    std::cout << "w :\n" << w_mat << '\n';
#endif

    JntArray q_min(7);
    JntArray q_max(7);

    ArmJoints lower;
    ArmJoints upper;

    q_min(0) = 0.0;        q_max(0) = 0.0;
    q_min(1) = -M_PI/2;    q_max(1) = M_PI/2;
    q_min(2) = -M_PI;      q_max(2) = 0.0;
    q_min(3) = -M_PI/2;    q_max(3) = M_PI/2;
    q_min(4) = -M_PI;      q_max(4) = 0.0;
    q_min(5) = 0.0;    q_max(5) = M_PI/2;
    q_min(6) = -M_PI/2;    q_max(6) = M_PI/2;
    for(int i=1; i<7; i++)
    {
        lower.q[i] = q_min(i);
        upper.q[i] = q_max(i);
    }

    left_arm.fk_solver = new ChainFkSolverPos_recursive(left_arm.chain);
    left_arm.ikvel_solver = new ChainIkSolverVel_pinv(left_arm.chain);
    left_arm.ikvel_solver_wdls = new ChainIkSolverVel_wdls(left_arm.chain);
    left_arm.ikpos_solver_lma = new ChainIkSolverPos_LMA(left_arm.chain,w_mat);
    left_arm.lower_limit = lower;
    left_arm.upper_limit = upper;

    right_arm.chain.addSegment(Segment(Joint("j0",Joint::None)));
    right_arm.chain.addSegment(Segment(Joint("j1",Joint::RotY),Frame(Vector(0.0,-10.0,0.0))));
    right_arm.chain.addSegment(Segment(Joint("j2",Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
    right_arm.chain.addSegment(Segment(Joint("j3",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    right_arm.chain.addSegment(Segment(Joint("j4",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
    right_arm.chain.addSegment(Segment(Joint("j5",Joint::RotZ),Frame(Vector(0.0,0.0,-20.0))));
    right_arm.chain.addSegment(Segment(Joint("j6",Joint::RotY),Frame(Vector(0.0,0.0,0.0))));

    q_min(0) = 0.0;        q_max(0) = 0.0;
    q_min(1) = -M_PI/2;    q_max(1) = M_PI/2;
    q_min(2) = -M_PI;      q_max(2) = 0.0;
    q_min(3) = -M_PI/2;    q_max(3) = M_PI/2;
    q_min(4) = -M_PI;      q_max(4) = 0.0;
    q_min(5) = 0.0;    q_max(5) = M_PI/2;
    q_min(6) = -M_PI/2;    q_max(6) = M_PI/2;

    for(int i=1; i<7; i++)
    {
        lower.q[i] = q_min(i);
        upper.q[i] = q_max(i);
    }

    right_arm.fk_solver = new ChainFkSolverPos_recursive(right_arm.chain);
    right_arm.ikvel_solver = new ChainIkSolverVel_pinv(right_arm.chain);
    right_arm.ikvel_solver_wdls = new ChainIkSolverVel_wdls(right_arm.chain);
    right_arm.ikpos_solver_lma = new ChainIkSolverPos_LMA(right_arm.chain,w_mat);
    right_arm.lower_limit = lower;
    right_arm.upper_limit = upper;

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
    DinsowArmChain &chain = (arm == LEFT) ? left_arm : right_arm;
    p = forwardKinematic(q,chain);
    return p;
}

DinsowKinematic::ArmJoints DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, DinsowKinematic::ArmSelect_t arm, int retry)
{
    ArmJoints j;
    DinsowArmChain &chain = (arm == LEFT) ? left_arm : right_arm;
    j = inverseKinematic(p,chain);
#if 1
    std::vector<int> limits;
    while(!checkLimits(chain,j,limits))
    {
#if 1
        std::cout << "[InverseKinematic] retry..\n";
#endif
        randomizeJoints(chain,limits);
        j = inverseKinematic(p,chain);
        if(retry<=0)
            break;
        retry--;
    }
#endif
    if(checkLimits(chain,j,limits))
        apply(j,chain);
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

DinsowKinematic::Pose DinsowKinematic::forwardKinematic(const DinsowKinematic::ArmJoints &q, DinsowKinematic::DinsowArmChain &chain)
{

}

DinsowKinematic::ArmJoints DinsowKinematic::inverseKinematic(const DinsowKinematic::Pose &p, DinsowArmChain &chain)
{
    auto n = chain.chain.getNrOfJoints();
    std::stringstream ss;
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
#ifdef DEBUG_LMA
    chain.ikpos_solver_lma->display_information = true;
#endif
    auto err = chain.ikpos_solver_lma->CartToJnt(in_q,in_frame,out_q);
    ss << "IK : " << (err==0 ? "success\n" : (err==-1 ? "gradients towards joint is too small\n" : (err==-2 ? "joint position increments are too small\n" : "number of iteration is exceeded\n")));
#else
    auto err = chain.ikpos_solver_jl->CartToJnt(in_q,in_frame,out_q);
    ss << "IK : " << chain.ikpos_solver->strError(err) << '\n';
#endif
#ifdef DEBUG_IK
    std::cout << "[InverseKinematic] p : " << p.str() << std::endl
              << ss.str() << std::endl;
#endif
    for(int i=0; i<n; i++)
        joints.q[i] = out_q(i);
#ifndef USE_ROTATION
    joints.q[4] = p.rx;
    joints.q[5] = p.ry;
#endif
    normalize(joints);
    return joints;
}

void DinsowKinematic::apply(const DinsowKinematic::ArmJoints &q, DinsowKinematic::DinsowArmChain &chain, bool clip)
{
    for(int i=0; i<6; i++)
    {
        chain.joint.q[i] = q.q[i];
        if(chain.joint.q[i] < chain.lower_limit.q[i])
            chain.joint.q[i] = chain.lower_limit.q[i];
        else if(chain.joint.q[i] > chain.upper_limit.q[i])
            chain.joint.q[i] = chain.upper_limit.q[i];
    }
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

void DinsowKinematic::setJointLimits(DinsowKinematic::DinsowArmChain &chain, const ArmJoints &min, const ArmJoints &max)
{
    chain.lower_limit = min;
    chain.upper_limit = max;
}

void DinsowKinematic::randomizeJoints(DinsowKinematic::DinsowArmChain &chain, std::vector<int> joints)
{
    static std::random_device rd;
    static std::mt19937 twister(rd());
    for(size_t i=0; i<N_ARM_JOINTS; i++)
    {
        if(joints.at(i)==0)
            continue;
        std::uniform_real_distribution<> dis(chain.lower_limit.q[i],
                                             chain.upper_limit.q[i]);
        chain.joint.q[i] = dis(twister);
    }
}

bool DinsowKinematic::checkLimits(const DinsowKinematic::DinsowArmChain &chain, const DinsowKinematic::ArmJoints &joint, std::vector<int> &limit)
{
    auto ret = true;
    limit.clear();
    limit.resize(N_ARM_JOINTS,0);
    for(size_t i=0; i<N_ARM_JOINTS; i++)
    {
        if(joint.q[i] > chain.upper_limit.q[i])
        {
            limit.at(i) = 1;
            ret = false;
        }
        else if(joint.q[i] < chain.lower_limit.q[i])
        {
            limit.at(i) = -1;
            ret = false;
        }
    }
    return ret;
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

