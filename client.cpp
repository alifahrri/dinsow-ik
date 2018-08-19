#include "client.h"
#include <iostream>
#include <sstream>
#include <utility>

TargetClient::TargetClient(quint16 port, QObject *parent)
    : QObject(parent), 
      socket(new QUdpSocket(this))
{
    this->port = port;
    sender = QHostAddress::LocalHost;
    socket->bind(sender,port);
    connect(this->socket, SIGNAL(readyRead()),
            this, SLOT(readyRead()));
    std::cout << "Ready\n";
}

void TargetClient::readyRead()
{
    dinsow_ik::Target target;
    QByteArray data;
    data.resize(socket->pendingDatagramSize());
    socket->readDatagram(data.data(), data.size(), &sender, &port);
    auto parsed = target.ParseFromArray(data.data(), data.size());
    std::string str = target.DebugString();
    std::cout << "[Target Client]===========================" << std::endl;
    std::cout << str << std::endl;
    std::cout << "==========================================" << std::endl;
    ikRequest(target.x(), target.y(), target.z());
}
