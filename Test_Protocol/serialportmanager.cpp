#include "serialportmanager.h"
#include "serialmanager.h"
#include "vtolprotocol.h"
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

void SerialPortManager::handleReadyRead()
{
    //m_readData.append(m_serialPort->readAll());
    QByteArray arr = m_serialPort->readAll();
    if (arr.size() < 3) {
        return;
    }
    SerialPacket packet;
    packet._checkSum = arr[0];
    packet._msgType = arr[1];
    packet._dataLength = arr[2];
    memcpy(packet.buffer, arr.data()+3, packet._dataLength);

    vtol_protocol::ProtocolMsg msg;
    auto code = vtol_protocol::Parser::parse(packet, msg);
    std::cout << (int)code << " " << "bytelen: " << packet._dataLength << " elementLen: " << msg.lenData << "\n";
    for (int i = 0; i < msg.lenData; i++)
        std::cout << msg.data[i].number << " ";
    std::cout << "\n";

    std::cout << "Get\n";

    //qDebug() << m_serialPort->read(20) << "\n";
    //if (!m_timer.isActive())
    //    m_timer.start(5000);
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
