#ifndef DINSOWKINEMATIC_H
#define DINSOWKINEMATIC_H

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>
#include <string>

#define N_ARM_JOINTS 6

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
        std::string str() const;
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
        KDL::ChainIkSolverVel_wdls *ikvel_solver_wdls;
        KDL::ChainIkSolverPos_LMA *ikpos_solver_lma;
        KDL::ChainIkSolverPos_NR *ikpos_solver;
        KDL::ChainIkSolverPos_NR_JL *ikpos_solver_jl;
        JointValues<n> joint;
        JointValues<n> lower_limit;
        JointValues<n> upper_limit;
    };

    typedef JointValues<N_ARM_JOINTS> ArmJoints;
    typedef DinsowChain<N_ARM_JOINTS> DinsowArmChain;

public:
    DinsowKinematic();
    Pose forwardKinematic(const ArmJoints &q, ArmSelect_t arm);
    ArmJoints inverseKinematic(const Pose &p, ArmSelect_t arm, int retry = 10);
    ArmJoints joints(ArmSelect_t arm);

private:
    Pose forwardKinematic(const ArmJoints &q, DinsowArmChain &chain);
    ArmJoints inverseKinematic(const Pose &p, DinsowArmChain &chain);
    void apply(const ArmJoints &q, DinsowArmChain &chain, bool clip = true);
    void normalize(ArmJoints &arm_joints);
    void setJointLimits(DinsowArmChain &chain, const ArmJoints &min, const ArmJoints &max);
    void randomizeJoints(DinsowArmChain &chain, std::vector<int> joints = std::vector<int>(N_ARM_JOINTS,1));
    bool checkLimits(const DinsowArmChain &chain, const ArmJoints &joint, std::vector<int> &limit);

private:
    DinsowArmChain left_arm;
    DinsowArmChain right_arm;
};

#endif // DINSOWKINEMATIC_H
