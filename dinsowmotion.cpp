#include "dinsowmotion.h"

//#define TEST
#ifdef TEST
#include <iostream>
#endif

DinsowMotion::DinsowMotion(DinsowKinematic &dinsow)
    : _dinsow(dinsow)
{
#ifdef TEST
    std::cout << "[DinsowMotion] linear test\n";
    DinsowKinematic::Pose p0({0.0,0.0,0.0,
                              0.0,0.0,0.0});
    DinsowKinematic::Pose p1({1.0,1.0,1.0,
                              1.0,1.0,1.0});
    Timer timer({0.0,1.0,0.5});
    std::cout << "p0 : " << p0.str()
              << std::endl
              << "p1 : " << p1.str()
              << std::endl;
    auto p = linear(p0,p1,timer);
    std::cout << "timer : " << timer.str() << std::endl;
    std::cout << "p : " << p.str() << std::endl;
#endif
}

DinsowMotion::DinsowArms DinsowMotion::motion(const DinsowKinematic::Pose &pose0, const DinsowKinematic::Pose &pose1, const DinsowMotion::Timer &timer)
{
    DinsowArms arm_joints;
    auto p = linear(pose0,pose1,timer);
    arm_joints.left = _dinsow.inverseKinematic(p,DinsowKinematic::LEFT);
    arm_joints.right = _dinsow.inverseKinematic(p,DinsowKinematic::RIGHT);
    return arm_joints;
}

DinsowKinematic::Pose DinsowMotion::linear(const DinsowKinematic::Pose p0, const DinsowKinematic::Pose &p1, const DinsowMotion::Timer &timer)
{
    auto x_min = p0.x;
    auto x_max = p1.x;
    auto y_min = p0.y;
    auto y_max = p1.y;
    auto z_min = p0.z;
    auto z_max = p1.z;
    auto rx_min = p0.rx;
    auto rx_max = p1.rx;
    auto ry_min = p0.ry;
    auto ry_max = p1.ry;
    auto rz_min = p0.rz;
    auto rz_max = p1.rz;

    auto x = linear(x_min,x_max,timer);
    auto y = linear(y_min,y_max,timer);
    auto z = linear(z_min,z_max,timer);
    auto rx = linear(rx_min,rx_max,timer);
    auto ry = linear(ry_min,ry_max,timer);
    auto rz = linear(rz_min,rz_max,timer);

    return DinsowKinematic::Pose({x,y,z,rx,ry,rz});
}

double DinsowMotion::linear(double y_min, double y_max, const DinsowMotion::Timer &timer)
{
    auto m = (y_max - y_min)/(timer.final_time-timer.init_time);
    auto dt = timer.current_time - timer.init_time;
    auto y = (m*dt) + y_min;
    return y;
}

std::__cxx11::string DinsowMotion::Timer::str() const
{
    std::stringstream ss;
    ss << "(t0 : " << this->init_time << "),"
       << "(t1 : " << this->final_time << "),"
       << "(t : " << this->current_time << ")";
    return ss.str();
}
