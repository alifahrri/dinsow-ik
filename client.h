#ifndef CLIENT_H
#define CLIENT_H

#include <QUdpSocket>
#include "message.pb.h"

struct Target
{
    float x;
    float y;
    float z;
};

class TargetClient : public QObject
{
    Q_OBJECT
public :
    explicit TargetClient(quint16 port, QObject *parent = nullptr);
    virtual ~TargetClient() {}
    TargetClient() {}
    Target getTarget() { return target; }
private :
    QUdpSocket *socket;
    QHostAddress sender;
    quint16 port;
    Target target;
public slots : 
    void readyRead();
signals:
    void ikRequest(double,double,double);
};

#endif
