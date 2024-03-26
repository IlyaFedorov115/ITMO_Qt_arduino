#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H
#include "serialmanager.h"
#include <QByteArray>
#include <QSerialPort>
#include <QTextStream>
#include <QTimer>
#include <QObject>

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

private slots:
    void handleReadyRead();
    //void handleReadyRead();
    void handleTimeout();
    void handleError(QSerialPort::SerialPortError error);


private:
    void _close();
    SerialPacketManager m_packetManager;
    QSerialPort *m_serialPort = nullptr;
    QByteArray m_readData;
    QTextStream m_standardOutput;
    QTimer m_timer;
};

#endif // SERIALPORTMANAGER_H
