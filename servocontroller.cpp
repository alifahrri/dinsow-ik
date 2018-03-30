#include "servocontroller.h"
#include "utility.hpp"
#include "dinsowkinematic.h"
#include <iostream>
#include <sstream>
#include <cmath>

#define MX64_SERVO
#define CONTROLLER_TEST
#define DEBUG
#define BAUD_RATE           (1000000)
#define PROTOCOL_VERSION    (1.0)
#define SERVO_ID            ("1,2,3,4,5,6")
#define TO_DEGREE           (180.0/M_PI)
#define TO_RADIAN           (M_PI/180.0)
#define TO_MX64_DEG         (4096/360)
#define TO_MX64_RAD         (4096/M_PI)
#define RX64_TO_DEG         (0.29)
#define RX64_CENTER         (512)
#define MX64_TO_DEG         (0.088)
#define MX64_CENTER         (2048)

//mx64
#define ADDR_GOAL_POS       30
#define LEN_GOAL_POS        2
#define ADDR_PRESENT_POS    36
#define LEN_PRESENT_POS     2
#define ADDR_TORQUE_EN      24
#define LEN_TORQUE_EN       1

#define TORQUE_ENABLE       1
#define TORQUE_DISABLE      0

#define ADDR_CW_LIMIT       (6)
#define ADDR_CCW_LIMIT      (8)
#define ADDR_MAX_TORQUE     (14)
#define ADDR_TURN_OFFSET    (20)
#define ADDR_RES_DIVIDER    (22)

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

    jointPresentPos.resize(servoIDs.size());
    jointPresentPosDeg.resize(servoIDs.size());
    gearRatio.resize(servoIDs.size(),1.0);
    rotation.resize(servoIDs.size(),1);
}

ServoController::~ServoController()
{
    stop();
}

std::vector<double> ServoController::presentPosDeg()
{
    return jointPresentPosDeg;
}

std::vector<int> ServoController::presentPos()
{
    return jointPresentPos;
}

ServoController::EEPROMSettings ServoController::readEEPROM(int id)
{
    EEPROMSettings ret;
    uint16_t data;
    uint8_t dxl_error;
    int dxl_comm_result = COMM_TX_FAIL;
    mutex.lock();

    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler,id,ADDR_CW_LIMIT,&data,&dxl_error);
    if((dxl_error == 0) && (dxl_comm_result == COMM_SUCCESS))
        ret.cw_angle_limit = (uint)data;
    else
        ret.cw_angle_limit = NAN;

    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler,id,ADDR_CCW_LIMIT,&data,&dxl_error);
    if((dxl_error == 0) && (dxl_comm_result == COMM_SUCCESS))
        ret.ccw_angle_limit = (uint)data;
    else
        ret.ccw_angle_limit = NAN;

    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler,id,ADDR_MAX_TORQUE,&data,&dxl_error);
    if((dxl_error == 0) && (dxl_comm_result == COMM_SUCCESS))
        ret.max_torque = (uint)data;
    else
        ret.max_torque = NAN;

    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler,id,ADDR_TURN_OFFSET,&data,&dxl_error);
    if((dxl_error == 0) && (dxl_comm_result == COMM_SUCCESS))
        ret.multi_turn_offset = (int16_t)(data&0xFFFF);
    else
        ret.multi_turn_offset = NAN;

    uint8_t data8;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler,id,ADDR_RES_DIVIDER,&data8,&dxl_error);
    if((dxl_error == 0) && (dxl_comm_result == COMM_SUCCESS))
        ret.resolution_divider = (int)data8;
    else
        ret.resolution_divider = NAN;

    mutex.unlock();
    return ret;
}

bool ServoController::writeEEPROM(const EEPROMSettings &settings, int id)
{
    uint8_t dxl_error;
    uint16_t data;
    auto success = true;
    int dxl_comm_result = COMM_TX_FAIL;
    mutex.lock();

    data = settings.cw_angle_limit;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,id,ADDR_CW_LIMIT,data,&dxl_error);
    if((dxl_error != 0) || (dxl_comm_result != COMM_SUCCESS))
    {
        success = false;
        std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }

    data = settings.ccw_angle_limit;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,id,ADDR_CCW_LIMIT,data,&dxl_error);
    if((dxl_error != 0) || (dxl_comm_result != COMM_SUCCESS))
    {
        success = false;
        std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }

    data = settings.max_torque;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,id,ADDR_MAX_TORQUE,data,&dxl_error);
    if((dxl_error != 0) || (dxl_comm_result != COMM_SUCCESS))
    {
        success = false;
        std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }

    data = settings.multi_turn_offset;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,id,ADDR_TURN_OFFSET,data,&dxl_error);
    if((dxl_error != 0) || (dxl_comm_result != COMM_SUCCESS))
    {
        success = false;
        std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }

    data = settings.resolution_divider;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler,id,ADDR_RES_DIVIDER,data,&dxl_error);
    if((dxl_error != 0) || (dxl_comm_result != COMM_SUCCESS))
    {
        success = false;
        std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
    }

    mutex.unlock();

    return success;
}

void ServoController::setGearRatio(std::vector<double> ratio)
{
    for(size_t i=0; i<std::min(ratio.size(),gearRatio.size()); i++)
        gearRatio.at(i) = ratio.at(i);
}

void ServoController::setTorque(std::vector<bool> torque)
{
    for(size_t i=0; i<std::min(torque.size(),(size_t)6); i++)
        jointValues[i].torque = torque[i];
}

void ServoController::setGoalPos(std::vector<int> goal)
{
    for(size_t i=0; i<std::min(goal.size(),(size_t)6); i++)
        jointValues[i].value = goal[i];
}

void ServoController::setRotation(std::vector<int> rot)
{
    for(size_t i=0; i<std::min(rot.size(),rotation.size()); i++)
        rotation[i] = rot[i];
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
#ifndef CONTROLLER_TEST
        readJoints();
#endif
        mutex.lock();
        syncRead(read_str);
        syncWrite(write_str);
        mutex.unlock();
#ifdef DEBUG
        std::cout << read_str;
        std::cout << write_str;
#endif
        if(serial_cb)
            serial_cb();
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
    ss << "[syncRead]=========================\n";

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
        if((dxl_error == 0) && (dxl_comm_result == COMM_SUCCESS))
            jointPresentPos[i] = data;
        else
            jointPresentPos[i] = -1;
#if 0
        get_data_result = syncReader.isAvailable(servoIDs[i],ADDR_PRESENT_POS,LEN_PRESENT_POS);
        if(!get_data_result)
        {
            ss << "[ID:" << servoIDs[i] << "] syncRead failed\n";
            ret = false;
        }
#endif
    }

    processPresentPos();

    str = ss.str();
    return ret;
}

void ServoController::processPresentPos()
{
    for(size_t i=0; i<jointPresentPos.size(); i++)
    {
#ifdef MX64_SERVO
        if(jointPresentPos[i]>=0)
            jointPresentPosDeg[i] = rotation.at(i) * (jointPresentPos[i]-MX64_CENTER) * MX64_TO_DEG / gearRatio.at(i);
#else
        if(jointPresentPos[i]>=0)
            jointPresentPosDeg[i] = rotation.at(i) * (jointPresentPos[i]-RX64_CENTER) * RX64_TO_DEG / gearRatio.at(i);
#endif
        else
            jointPresentPosDeg[i] = NAN;
    }
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

    ss << "[syncWrite]=========================\n";

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
#ifndef CONTROLLER_TEST
    value = (int)(joint*TO_MX64_RAD+offset);
#endif
    for(size_t i=0; i<nbyte; i++)
    {
        servo_value[i] = (uint8_t)((value>>(i*8)&0xFF));
        std::cout << "[ServoController] servo[" << i << "] : " << (size_t)(servo_value[i]) << '\n';
    }
    return servo_value;
}

std::__cxx11::string ServoController::EEPROMSettings::str()
{
    std::stringstream ss;
    ss << "(CW : " << cw_angle_limit << "),"
       << "(CCW : " << ccw_angle_limit << "),"
       << "(MAX_TORQUE : " << max_torque << "),"
       << "(OFFSET : " << multi_turn_offset << "),"
       << "(DIVIDER : " << resolution_divider << ")";
    return ss.str();
}
