#ifndef DINSOWKINEMATIC_H
#define DINSOWKINEMATIC_H

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <vector>
#include <string>

class DinsowKinematic
{
public:
    enum ArmSelect_t
    {
        LEFT = 0,
        RIGHT = 1
    };

public:
    struct Pose
    {
        double x, y, z;
        double rx, ry, rz;
        std::string str();
    };

    template <int n>
    struct JointValues
    {
        double q[n];
        std::string str();
    };


    template <int n>
    struct DinsowChain
    {
        KDL::Chain chain;
        KDL::ChainFkSolverPos_recursive *fk_solver;
        KDL::ChainIkSolverVel_pinv *ikvel_solver;
        KDL::ChainIkSolverPos_NR *ikpos_solver;
        JointValues<n> joint;
    };

    typedef JointValues<6> ArmJoints;
    typedef DinsowChain<6> DinsowArmChain;

public:
    DinsowKinematic();
    Pose forwardKinematic(const ArmJoints &q, ArmSelect_t arm);
    ArmJoints inverseKinematic(const Pose &p, ArmSelect_t arm);
    ArmJoints joints(ArmSelect_t arm);

private:
    Pose forwardKinematic(const ArmJoints &q, DinsowArmChain &chain);
    ArmJoints inverseKinematic(const Pose &p, DinsowArmChain &chain);
    Pose forwardKinematic(const ArmJoints &q);
    ArmJoints inverseKinematic(const Pose &p, const ArmJoints &init_q);

private:
    DinsowArmChain left_arm;
    DinsowArmChain right_arm;

    KDL::Chain lhand_chain;
    KDL::ChainFkSolverPos_recursive *lhand_chain_fk;
    KDL::ChainIkSolverVel_pinv *lhand_ik_vel;
    KDL::ChainIkSolverPos_NR *lhand_ik_pos;
};

#endif // DINSOWKINEMATIC_H
