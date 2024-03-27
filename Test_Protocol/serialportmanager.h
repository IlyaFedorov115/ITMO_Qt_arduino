#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H
#include "serialmanager.h"
#include <QByteArray>
#include <QSerialPort>
#include <QTextStream>
#include <QTimer>
#include <QObject>
#include "vtolprotocol.h"

class SerialPortManager : public QObject
{
    Q_OBJECT

public:
    explicit SerialPortManager(QSerialPort *serialPort, QObject *parent = nullptr);
    ~SerialPortManager();
    void sendTimerStepHW(unsigned time_step = 100);
    void sendPwmSignal(unsigned pwm);
    void sendStartSim();
    void sendStopSim();
    void sendAngleType(vtol_protocol::MsgProps::ANGLE_TYPE type);

private slots:
    void handleReadyRead();
    //void handleReadyRead();
    void handleTimeout();
    void handleError(QSerialPort::SerialPortError error);


private:
    void _close();
    vtol_protocol::ProtocolMsg _prepareMsg(vtol_protocol::MsgProps::MSG_TYPE type);
    void _sendMessage(vtol_protocol::ProtocolMsg& msg);
    SerialPacketManager m_packetManager;
    QSerialPort *m_serialPort = nullptr;
    QByteArray m_readData;
    QTextStream m_standardOutput;
    QTimer m_timer;
};

#endif // SERIALPORTMANAGER_H
