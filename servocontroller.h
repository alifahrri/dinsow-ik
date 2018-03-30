#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

#define IK_TEST

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
        int low_limit;
        int high_limit;
        double speed;
        uint mov_speed;
        uint8_t servo_value[nbyte];
        uint8_t *operator()();
    };

    struct EEPROMSettings
    {
        int cw_angle_limit;
        int ccw_angle_limit;
        int max_torque;
        int multi_turn_offset;
        int resolution_divider;
        std::string str();
    };

    typedef ServoJointValue<2> MX64ServoJoint;

public:
    ServoController(std::__cxx11::string port = std::string("/dev/ttyUSB0"));
    ~ServoController();

    std::vector<double> presentPosDeg();
    EEPROMSettings readEEPROM(int id);
    std::vector<int> presentPos();

    bool writeEEPROM(const EEPROMSettings &settings, int id);
    void setGearRatio(std::vector<double> ratio);
    void setTorque(std::vector<bool> torque);
    void setGoalPos(std::vector<int> goal);
    void setRotation(std::vector<int> rot);
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
    void readDinsowJoints();
    void processJointPos();
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
    std::vector<int> rotation;
    std::atomic_bool running;
    std::mutex mutex;
    std::thread thread;
};

#endif // SERVOCONTROLLER_H
