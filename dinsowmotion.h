#ifndef DINSOWMOTION_H
#define DINSOWMOTION_H

#include "dinsowkinematic.h"

class DinsowMotion
{
public:
    struct Timer
    {
        double init_time;
        double final_time;
        double current_time;
        std::string str() const;
    };
    struct DinsowArms
    {
        DinsowKinematic::ArmJoints left;
        DinsowKinematic::ArmJoints right;
    };

public:
    DinsowMotion(DinsowKinematic &dinsow);
    DinsowArms motion(const DinsowKinematic::Pose &pose0, const DinsowKinematic::Pose &pose1, const Timer &timer);

private:
    DinsowKinematic &_dinsow;

private:
    DinsowKinematic::Pose linear(const DinsowKinematic::Pose p0, const DinsowKinematic::Pose &p1, const Timer &timer);
    double linear(double y_min, double y_max, const Timer &timer);
};

#endif // DINSOWMOTION_H
