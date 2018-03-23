#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <thread>
#include <atomic>

class DinsowKinematic;

class ServoController
{
public:
    template <int nbyte>
    struct ServoJointValue
    {
        double joint;
        int value;
        int offset;
        bool torque;
        uint8_t servo_value[nbyte];
        uint8_t *operator()();
    };

    typedef ServoJointValue<2> MX64ServoJoint;

public:
    ServoController(std::__cxx11::string port = std::string("/dev/ttyUSB0"));
    ~ServoController();
    void setTorque(std::vector<bool> torque);
    void setGoalPos(std::vector<int> goal);
    bool open(std::string port);
    bool open();
    void start();
    void stop();

public:
    DinsowKinematic *dinsow = nullptr;

private:
    void loop();
    void readJoints();
    bool enableTorque(bool en, std::string *str = nullptr);
    bool syncRead(std::string &str);
    bool syncWrite(std::string &str);

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite syncWriter;
    dynamixel::GroupSyncWrite syncTorqueWriter;
    dynamixel::GroupSyncRead syncReader;
    std::vector<MX64ServoJoint> jointValues;
    std::vector<int> servoIDs;
    std::atomic_bool running;
    std::thread thread;
};

#endif // SERVOCONTROLLER_H
