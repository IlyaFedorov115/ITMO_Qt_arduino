#include "serialportmanager.h"
#include "serialmanager.h"

#include <QDebug>
#include <iostream>


SerialPortManager::SerialPortManager(QSerialPort *serialPort, QObject *parent):
    QObject(parent),
    m_serialPort(serialPort),
    m_standardOutput(stdout)
{
    connect(m_serialPort, &QSerialPort::readyRead, this, &SerialPortManager::handleReadyRead);
    connect(m_serialPort, &QSerialPort::errorOccurred, this, &SerialPortManager::handleError);
    //connect(&m_timer, &QTimer::timeout, this, &SerialPortReaderWriter::handleTimeout);
    qDebug() << "Create Serial reader\n";
    m_readData.reserve(300);
    //m_timer.start(5000);
}

SerialPortManager::~SerialPortManager()
{
    _close();
}

void SerialPortManager::_sendMessage(vtol_protocol::ProtocolMsg &msg)
{
    auto packet = vtol_protocol::Parser::parse2Serial(msg);
    packet._checkSum = m_packetManager.calcCheckSum(packet);
    m_serialPort->write((char*)&packet, packet.getFullPacketSize());
    m_serialPort->flush();
}

vtol_protocol::ProtocolMsg SerialPortManager::_prepareMsg(vtol_protocol::MsgProps::MSG_TYPE type)
{
    //vtol_protocol::ProtocolMsg msg;
    //msg.type = type;
}

void SerialPortManager::sendTimerStepHW(unsigned time_step)
{
    vtol_protocol::ProtocolMsg msg(vtol_protocol::MsgProps::MSG_TYPE::SET_BY_TIMER);
    msg.data[0].number = time_step;

    this->_sendMessage(msg);
}

void SerialPortManager::sendPwmSignal(unsigned pwm)
{
    qDebug() << "===SEND PWM ====";
    if (pwm < 1200 || pwm > 2000) qDebug() << "Pwm error\n";
    vtol_protocol::ProtocolMsg msg(vtol_protocol::MsgProps::MSG_TYPE::PWM_SIGNAL);
    msg.data[0].number = pwm;

    this->_sendMessage(msg);
}

void SerialPortManager::sendStartSim()
{
    vtol_protocol::ProtocolMsg msg(vtol_protocol::MsgProps::MSG_TYPE::START_SIM);

    this->_sendMessage(msg);
}

void SerialPortManager::sendStopSim()
{
    vtol_protocol::ProtocolMsg msg(vtol_protocol::MsgProps::MSG_TYPE::STOP_SIM);

    this->_sendMessage(msg);
}

void SerialPortManager::sendAngleType(vtol_protocol::MsgProps::ANGLE_TYPE type)
{
    vtol_protocol::ProtocolMsg msg(vtol_protocol::MsgProps::MSG_TYPE::SET_ANGLE_TYPE);
    msg.data[0].number = static_cast<int>(type);
    this->_sendMessage(msg);
}

void SerialPortManager::handleReadyRead()
{
    // if (m_serialPort->bytesAvailable() < 3)
    // m_serialPort->skip(m_serialPort->bytesAvailable())

    QByteArray arr = m_serialPort->readAll();
    if (arr.size() < 3) {
        emit sig_GetInvalidPacket();
        return;
    }
    // read packet
    SerialPacket packet;
    packet._checkSum = arr[0];
    packet._msgType = arr[1];
    packet._dataLength = arr[2];
    memcpy(packet.buffer, arr.data()+3, packet._dataLength);

    // check packet is valid
    if(!this->m_packetManager.isValidPacket(packet)){
        // bad packet
        qDebug() << "get packet with wring crc";
        emit sig_GetInvalidPacket();
        return;
    }

    vtol_protocol::ProtocolMsg msg;
    auto code = vtol_protocol::Parser::parse(packet, msg);
    if (code == vtol_protocol::Parser::PARSE_CODE::SUCCESS){
        this->_handleMessage(msg);
    } else {
        qDebug() << "Get invalid msg (parse)";
        emit sig_GetInvalidMsg(code);
    }
    qDebug() << "Port get packet";

}

void SerialPortManager::_handleMessage(vtol_protocol::ProtocolMsg msg)
{
    qDebug() << "Analyse";
    switch (msg.type) {
    case vtol_protocol::MsgProps::MSG_TYPE::EULER_ANGLE:
        emit sig_GetEulerAngle(msg.data[0].number, msg.data[1].number, msg.data[2].number);
        break;
    case vtol_protocol::MsgProps::MSG_TYPE::YAW_PITCH_ROLL:
        emit sig_GetYawPitchRoll(msg.data[0].number, msg.data[1].number, msg.data[2].number);
        break;
    case vtol_protocol::MsgProps::MSG_TYPE::QUART_ANGLE:
        qDebug() << "GET QUART";
        emit sig_GetQuartAngle(msg.data[0].number, msg.data[1].number,
                msg.data[2].number, msg.data[3].number);
        break;
    case vtol_protocol::MsgProps::MSG_TYPE::RAW_GYRO_ACCEL:
        emit sig_GetRawAngle(msg.data[0].number, msg.data[1].number, msg.data[2].number,
                msg.data[3].number, msg.data[4].number, msg.data[5].number);
        break;
    default:
        qDebug() << "Unknown msg";
        break;
    }

}

void SerialPortManager::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::ReadError) {
        m_standardOutput << QObject::tr("An I/O error occurred while reading "
                                        "the data from port %1, error: %2")
                            .arg(m_serialPort->portName())
                            .arg(m_serialPort->errorString())
                         << "\n";
    }
}



void SerialPortManager::_close()
{
    this->sendStopSim();
    m_serialPort->close();
}






void SerialPortManager::handleTimeout()
{
    /*
    if (m_readData.isEmpty()) {
        m_standardOutput << QObject::tr("No data was currently available "
                                        "for reading from port %1")
                            .arg(m_serialPort->portName())
                         << "\n";
    } else {
        m_standardOutput << QObject::tr("Data successfully received from port %1")
                            .arg(m_serialPort->portName())
                         << "\n";
        m_standardOutput << m_readData << "\n";
    }

    QCoreApplication::quit();
    */
}
