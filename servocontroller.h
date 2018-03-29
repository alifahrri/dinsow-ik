#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <thread>
#include <atomic>
#include <functional>

class DinsowKinematic;

class ServoController
{
public:
    typedef std::function<void()> SerialCallback;

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
    std::vector<double> presentPosDeg();
    std::vector<int> presentPos();
    void setGearRatio(std::vector<double> ratio);
    void setTorque(std::vector<bool> torque);
    void setGoalPos(std::vector<int> goal);
    bool open(std::string port);
    bool open();
    void start();
    void stop();

public:
    DinsowKinematic *dinsow = nullptr;
    SerialCallback serial_cb = nullptr;

private:
    bool enableTorque(bool en, std::string *str = nullptr);
    bool syncWrite(std::string &str);
    bool syncRead(std::string &str);
    void processPresentPos();
    void readJoints();
    void loop();

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite syncWriter;
    dynamixel::GroupSyncWrite syncTorqueWriter;
    dynamixel::GroupSyncRead syncReader;
    std::vector<MX64ServoJoint> jointValues;
    std::vector<double> gearRatio;
    std::vector<double> jointPresentPosDeg;
    std::vector<int> jointPresentPos;
    std::vector<int> servoIDs;
    std::atomic_bool running;
    std::thread thread;
};

#endif // SERVOCONTROLLER_H
