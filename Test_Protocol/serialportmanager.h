#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H
#include "serialmanager.h"
#include <QByteArray>
#include <QSerialPort>
#include <QTextStream>
#include <QTimer>
#include <QObject>
#include <vector>
#include "vtolprotocol.h"

class IHandlerAngleReceive {
public:
    virtual ~IHandlerAngleReceive() {}
    virtual void handleYawPitchRoll(float y, float p, float r) = 0;
};

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
    void setHandlerAngle(IHandlerAngleReceive* handler);


private slots:
    void handleReadyRead();
    //void handleReadyRead();
    void handleTimeout();
    void handleError(QSerialPort::SerialPortError error);


signals:
    void sig_GetQuartAngle(float w, float x, float y, float z);
    void sig_GetYawPitchRoll(float y, float p, float r);
    void sig_GetRawAngle(float accelX, float accelY, float accelZ,
                         float gyroX, float gyroY, float gyroZ);
    void sig_GetEulerAngle(float x, float y, float z);

    void sig_GetInvalidMsg(vtol_protocol::Parser::PARSE_CODE code);
    void sig_GetInvalidPacket();

private:
    void _handleMessage(vtol_protocol::ProtocolMsg msg);
    void _close();
    vtol_protocol::ProtocolMsg _prepareMsg(vtol_protocol::MsgProps::MSG_TYPE type);
    void _sendMessage(vtol_protocol::ProtocolMsg& msg);
    SerialPacketManager m_packetManager;
    QSerialPort *m_serialPort = nullptr;
    QByteArray m_readData;
    QTextStream m_standardOutput;
    QTimer m_timer;

    std::vector<IHandlerAngleReceive*> _handlersAngle;
};

#endif // SERIALPORTMANAGER_H
