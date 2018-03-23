#include "servocontroller.h"
#include "utility.hpp"
#include "dinsowkinematic.h"
#include <iostream>
#include <sstream>
#include <cmath>

#define CONTROLLER_TEST
#define DEBUG
#define BAUD_RATE           (1000000)
#define PROTOCOL_VERSION    (1.0)
#define SERVO_ID            ("1,2,3,4,5,6")
#define TO_DEGREE           (180.0/M_PI)
#define TO_RADIAN           (M_PI/180.0)
#define TO_MX64_DEG         (4096/360)
#define TO_MX64_RAD         (4096/M_PI)

//mx64
#define ADDR_GOAL_POS       30
#define LEN_GOAL_POS        2
#define ADDR_PRESENT_POS    36
#define LEN_PRESENT_POS     2
#define ADDR_TORQUE_EN      24
#define LEN_TORQUE_EN       1

#define TORQUE_ENABLE       1
#define TORQUE_DISABLE      0

ServoController::ServoController(std::string port)
    : portHandler(dynamixel::PortHandler::getPortHandler(port.c_str())),
      packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)),
      syncWriter(dynamixel::GroupSyncWrite(portHandler,packetHandler,ADDR_GOAL_POS,LEN_GOAL_POS)),
      syncTorqueWriter(dynamixel::GroupSyncWrite(portHandler,packetHandler,ADDR_TORQUE_EN,LEN_TORQUE_EN)),
      syncReader(dynamixel::GroupSyncRead(portHandler,packetHandler,ADDR_PRESENT_POS,LEN_PRESENT_POS))
{
    std::stringstream ss(std::string(SERVO_ID));

    int i;
    while(ss >> i)
    {
        servoIDs.push_back(i);
        if(ss.peek() == ',')
            ss.ignore();
    }
    jointValues.resize(servoIDs.size(),MX64ServoJoint({0.0,0,0,false}));
    for(size_t i=0; i<jointValues.size(); i++)
    {
        jointValues.at(i).servo_value[0] = 0;
        jointValues.at(i).servo_value[1] = 0;
    }
    for(size_t i=0; i<servoIDs.size(); i++)
        syncReader.addParam(servoIDs[i]);
    portHandler->setBaudRate(BAUD_RATE);
}

ServoController::~ServoController()
{
    stop();
}

void ServoController::setTorque(std::vector<bool> torque)
{
    for(size_t i=0; i<std::min(torque.size(),6); i++)
    {
        jointValues[i].torque = torque[i];
    }
}

void ServoController::setGoalPos(std::vector<int> goal)
{
    for(size_t i=0; i<std::min(goal.size(),6); i++)
    {
        jointValues[i].value = goal[i];
    }
}

bool ServoController::open(std::__cxx11::string port)
{
    portHandler->setPortName(port.c_str());
    return portHandler->openPort();
}

bool ServoController::open()
{
    return portHandler->openPort();
}

void ServoController::start()
{
    running = true;
    thread = std::thread(&ServoController::loop,this);
}

void ServoController::stop()
{
    running = false;
    if(thread.joinable())
        thread.join();
    portHandler->closePort();
}

bool ServoController::enableTorque(bool en, std::__cxx11::string *str)
{
    bool ret = true;
    for(size_t i=0; i<servoIDs.size(); i++)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,
                                                            servoIDs[i],
                                                            ADDR_TORQUE_EN,
                                                            (en?TORQUE_ENABLE:TORQUE_DISABLE),
                                                            &dxl_error);
        (*str) += "[enableTorque] (ID:";
        (*str) += std::to_string(servoIDs[i]);
        (*str) += ")";
        if(dxl_comm_result != COMM_SUCCESS)
        {
            ret = false;
            if(str)
                (*str) += packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if(dxl_error != 0)
        {
            ret = false;
            if(str)
                (*str) += packetHandler->getRxPacketError(dxl_error);
        }
        else if(str)
            (*str) += "success";
        (*str) += '\n';
    }
    return ret;
}

void ServoController::loop()
{
    std::string str;
    std::cout << str << '\n';
    while(running)
    {
        utility::timer timer;
        std::string read_str, write_str;
        readJoints();
        syncRead(read_str);
        syncWrite(write_str);
#ifdef DEBUG
        std::cout << read_str;
        std::cout << write_str;
#endif
        timer.sleep(33.0);
    }
}

void ServoController::readJoints()
{
#if 1
    std::cout << "[ReadJoints] \n";
#endif
    if(!dinsow)
        return;
    auto l_joints = dinsow->joints(DinsowKinematic::LEFT);
    auto r_joints = dinsow->joints(DinsowKinematic::RIGHT);
    for(size_t i=0; i<6; i++)
    {
        jointValues.at(i).joint = l_joints.q[i];
        jointValues.at(i)();
#if 1
        static auto servo2str = [](const MX64ServoJoint &servo)->std::string
        {
            std::stringstream ss;
            ss << "(" << servo.joint    << ")"
               << "(" << servo.value    << ")"
               << "(" << servo.offset   << ")";

            ss << "(";
            for(size_t i=0; i<2; i++)
                ss << (int)servo.servo_value[i] << ((i==0)?",":")");
            return ss.str();
        };

        std::cout << "joint->(ID:" << servoIDs[i] << ") = "
                  << servo2str(jointValues.at(i)) << '\n';
#endif
    }
}

bool ServoController::syncRead(std::__cxx11::string &str)
{
    bool ret = true;
    bool get_data_result = false;
    std::stringstream ss;
    int dxl_comm_result = COMM_TX_FAIL;
    ss << "[syncRead]======================\n";

#if 0 //protocol 2.0 only
    dxl_comm_result = syncReader.txRxPacket();


    if(dxl_comm_result != COMM_SUCCESS)
    {
        ret = false;
        ss << packetHandler->getTxRxResult(dxl_comm_result);
        ss << "\n";
    }
    else
        ss << "comm successful\n";
#endif

    for(size_t i=0; i<servoIDs.size(); i++)
    {
        uint16_t data;
        uint8_t dxl_error;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler,servoIDs[i],ADDR_PRESENT_POS,&data,&dxl_error);
        ss << "[ID:" << servoIDs[i] << "] "
           << packetHandler->getTxRxResult(dxl_comm_result) << " "
           << packetHandler->getRxPacketError(dxl_error) << " "
           << data << '\n';
#if 0
        get_data_result = syncReader.isAvailable(servoIDs[i],ADDR_PRESENT_POS,LEN_PRESENT_POS);
        if(!get_data_result)
        {
            ss << "[ID:" << servoIDs[i] << "] syncRead failed\n";
            ret = false;
        }
#endif
    }
    str = ss.str();
    return ret;
}

bool ServoController::syncWrite(std::__cxx11::string &str)
{
    bool ret = true;
    std::stringstream ss;
    syncWriter.clearParam();
    syncTorqueWriter.clearParam();
    for(size_t i=0; i<servoIDs.size(); i++)
    {
        uint8_t torque_data = jointValues[i].torque ? TORQUE_ENABLE : TORQUE_DISABLE;
        syncTorqueWriter.addParam(servoIDs[i],&torque_data);
        if(jointValues[i].torque)
            syncWriter.addParam(servoIDs[i],jointValues[i]());
    }

    ss << "[syncWrite]======================\n";

    int dxl_torque_comm_result = syncTorqueWriter.txPacket();
    if(dxl_torque_comm_result != COMM_SUCCESS)
    {
        ret = false;
        ss << packetHandler->getTxRxResult(dxl_torque_comm_result);
        ss << "\n";
    }
    else ss << "torque comm successful\n";

    int dxl_comm_result = syncWriter.txPacket();
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ret = false;
        ss << packetHandler->getTxRxResult(dxl_comm_result);
        ss << "\n";
    }
    else ss << "goal pos comm successful\n";

    str = ss.str();
    return ret;
}

template<int nbyte>
uint8_t *ServoController::ServoJointValue<nbyte>::operator()()
{
    value = (int)(joint*TO_MX64_RAD+offset);
    for(size_t i=0; i<nbyte; i++)
        servo_value[i] = (uint8_t)((value>>i)&&0xFF);
    return servo_value;
}